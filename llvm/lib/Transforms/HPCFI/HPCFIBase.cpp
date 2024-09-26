//===-- HPCFIBase.cpp  ----------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#include "HPCFIBase.h"

#include "llvm/IR/InlineAsm.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/FileSystem.h"

#include <fstream>

#include "MLTAPass/MLTA.h"

#include "DDA/ContextDDA.h"
#include "DDA/DDAClient.h"
#include "Util/Options.h"
#include "WPA/FlowSensitive.h"
#include "WPA/VersionedFlowSensitive.h"

using namespace llvm;

static cl::opt<std::string>
    CFIIgnoreFile("hpcfi-ignore", cl::desc("Specify input filename for functions that should "
                                           "be ignored by HPCFI."));

static cl::opt<std::string> CFIOutFile("hpcfi-output",
                                       cl::desc("Specify output filename. default is stdout."),
                                       cl::init("-"));

namespace HPCFI {

bool HPCFIBase::runOnModule(Module &M) {
  this->M = &M;

  std::error_code EC;
  OUT = new raw_fd_ostream(CFIOutFile.getValue(), EC, sys::fs::OpenFlags::OF_Append);
  OUT->SetUnbuffered();

  setFuncnameToFuncs();
  createCFIFailFunc();
  setExclCallSites();
  updateMLTAInfos();

  // get SVF graphs
  SVF::LLVMModuleSet::releaseLLVMModuleSet();
  SVF::SVFModule *SVFModule = SVF::LLVMModuleSet::getLLVMModuleSet()->buildSVFModule(M);
  SVFModule->buildSymbolTableInfo();
  Pag = SVF::SVFIRBuilder().build(SVFModule);
  Icfg = Pag->getICFG();
  Ander = SVF::AndersenWaveDiff::createAndersenWaveDiff(Pag);
  Svfg = SVF::SVFGBuilder().buildPTROnlySVFG(Ander);
  CallGraph = Svfg->getMSSA()->getPTA()->getPTACallGraph();

  return false;
}

void HPCFIBase::setExclCallSites() {
  // parse excluded call sites from CFIIgnoreFile
  // format: {line <space> filename <newline>}
  std::string S = CFIIgnoreFile.getValue();
  if (S.empty())
    return;

  *OUT << "ignoring all functions in " << S << "\n";

  std::ifstream File(S);
  int Line;
  std::string F;
  while (File >> Line >> F) {
    ExclCallSites.insert({Line, F});
  }
}

void HPCFIBase::updateMLTAInfos() {
  /*
    MLTA results are stored as LLVM IR metadata. For example, struct_infos are
    stored like this: !llvm.module.flags = !{..., !7, !11} !7 = !{i32 1,
    !"struct_infos_LinkTests3/foo.c0", !8} !8 = !{!"struct.S", !9, !10} !9 =
    !{i32 0} !10 = !{!"funcA", !"funcB"} !11 = !{i32 1,
    !"struct_infos_LinkTests3/foo.c1", !12} !12 = !{!"struct.S", !13, !14} !13 =
    !{i32 1} !14 = !{!"funcB"}
  */
  std::map<std::pair<std::string, int>, std::set<std::pair<std::string, unsigned>>>
      GepsInFptrArrArgs;
  std::map<std::pair<std::string, unsigned>, std::map<std::vector<int>, std::set<std::string>>>
      FptrarrArgInits;
  std::set<std::pair<std::pair<std::string, int>, std::pair<std::string, int>>> MemcpyAliases;
  if (NamedMDNode *ModuleFlags = M->getNamedMetadata("llvm.module.flags")) {
    for (unsigned I = 0; I < ModuleFlags->getNumOperands(); ++I) {
      MDNode *Flag = ModuleFlags->getOperand(I);
      MDString *ModuleId = dyn_cast<MDString>(Flag->getOperand(1));
      if (ModuleId->getString().startswith(MLTA_STRUCT_INFOS)) {
        MDNode *TypeOffsetsInfo = dyn_cast<MDNode>(Flag->getOperand(2));
        std::string StructType =
            dyn_cast<MDString>(TypeOffsetsInfo->getOperand(0))->getString().str();
        std::vector<int> Offsets;
        MDNode *OffsetsNode = dyn_cast<MDNode>(TypeOffsetsInfo->getOperand(1));
        for (size_t I = 0; I < OffsetsNode->getNumOperands(); I++) {
          int Offset = dyn_cast<ConstantInt>(
                           dyn_cast<ConstantAsMetadata>(OffsetsNode->getOperand(I))->getValue())
                           ->getSExtValue();
          Offsets.push_back(Offset);
        }
        MDNode *MltaTargets = dyn_cast<MDNode>(TypeOffsetsInfo->getOperand(2));
        for (size_t I = 0; I < MltaTargets->getNumOperands(); I++) {
          std::string FunctionName =
              dyn_cast<MDString>(MltaTargets->getOperand(I))->getString().str();
          mlta_infos[{StructType, Offsets}].insert(FunctionName);
        }
      } else if (ModuleId->getString().startswith(MLTA_GEPS_IN_FPTRARRARGS)) {
        MDNode *Metadata = dyn_cast<MDNode>(Flag->getOperand(2));
        std::string StructType = dyn_cast<MDString>(Metadata->getOperand(0))->getString().str();
        int Offset =
            dyn_cast<ConstantInt>(dyn_cast<ConstantAsMetadata>(Metadata->getOperand(1))->getValue())
                ->getSExtValue();
        MDNode *FunctionsToArgs = dyn_cast<MDNode>(Metadata->getOperand(2));
        for (size_t I = 0; I < FunctionsToArgs->getNumOperands(); I += 2) {
          std::string Function =
              dyn_cast<MDString>(FunctionsToArgs->getOperand(I))->getString().str();
          int Argno =
              dyn_cast<ConstantInt>(
                  dyn_cast<ConstantAsMetadata>(FunctionsToArgs->getOperand(I + 1))->getValue())
                  ->getSExtValue();
          GepsInFptrArrArgs[{StructType, Offset}].insert({Function, Argno});
        }
      } else if (ModuleId->getString().startswith(MLTA_FPTRARR_ARG_INITS)) {
        MDNode *Metadata = dyn_cast<MDNode>(Flag->getOperand(2));
        std::string Function = dyn_cast<MDString>(Metadata->getOperand(0))->getString().str();
        int Argno =
            dyn_cast<ConstantInt>(dyn_cast<ConstantAsMetadata>(Metadata->getOperand(1))->getValue())
                ->getSExtValue();
        MDNode *OffsetsToTargets = dyn_cast<MDNode>(Metadata->getOperand(2));
        for (size_t I = 0; I < OffsetsToTargets->getNumOperands(); I += 2) {
          MDNode *OffsetsMd = dyn_cast<MDNode>(OffsetsToTargets->getOperand(I));
          MDNode *TargetsMd = dyn_cast<MDNode>(OffsetsToTargets->getOperand(I + 1));
          std::vector<int> Offsets;
          std::set<std::string> Targets;
          for (size_t I = 0; I < OffsetsMd->getNumOperands(); I++) {
            int Offset = dyn_cast<ConstantInt>(
                             dyn_cast<ConstantAsMetadata>(OffsetsMd->getOperand(I))->getValue())
                             ->getSExtValue();
            Offsets.push_back(Offset);
          }
          for (size_t I = 0; I < TargetsMd->getNumOperands(); I++) {
            std::string Target = dyn_cast<MDString>(TargetsMd->getOperand(I))->getString().str();
            Targets.insert(Target);
          }
          FptrarrArgInits[{Function, Argno}][Offsets].insert(Targets.begin(), Targets.end());
        }
      } else if (ModuleId->getString().startswith(MLTA_MEMCPY_ALIASES)) {
        MDNode *Metadata = dyn_cast<MDNode>(Flag->getOperand(2));
        std::string DstStructName = dyn_cast<MDString>(Metadata->getOperand(0))->getString().str();
        int DstOffset =
            dyn_cast<ConstantInt>(dyn_cast<ConstantAsMetadata>(Metadata->getOperand(1))->getValue())
                ->getSExtValue();
        std::string SrcStructName = dyn_cast<MDString>(Metadata->getOperand(2))->getString().str();
        int SrcOffset =
            dyn_cast<ConstantInt>(dyn_cast<ConstantAsMetadata>(Metadata->getOperand(3))->getValue())
                ->getSExtValue();
        MemcpyAliases.insert({{DstStructName, DstOffset}, {SrcStructName, SrcOffset}});
      }
    }
  }

  for (auto &Entry : GepsInFptrArrArgs) {
    std::string StructName = Entry.first.first;
    int Offset = Entry.first.second;
    for (auto FunctionNameArgno : Entry.second) {
      std::map<std::vector<int>, std::set<std::string>> OffsetsToTargets =
          FptrarrArgInits[FunctionNameArgno];
      for (auto &OffsetsTargets : OffsetsToTargets) {
        std::vector<int> Offsets = {Offset};
        Offsets.insert(Offsets.end(), OffsetsTargets.first.begin(), OffsetsTargets.first.end());
        mlta_infos[{StructName, Offsets}].insert(OffsetsTargets.second.begin(),
                                                 OffsetsTargets.second.end());
      }
    }
  }

  for (auto MemcpyAlias : MemcpyAliases) {
    std::pair<std::string, int> Dst = MemcpyAlias.first;
    std::pair<std::string, int> Src = MemcpyAlias.second;
    std::set<std::vector<int>> PreciseSrcOffsets;

    for (auto &MltaInfo : mlta_infos) {
      auto Key = MltaInfo.first;
      if (Key.first == Src.first && Key.second[0] == Src.second) {
        PreciseSrcOffsets.insert(Key.second);
      }
    }

    for (auto PreciseSrcOffset : PreciseSrcOffsets) {
      std::vector<int> OffsetsDst = {Dst.second};
      OffsetsDst.insert(OffsetsDst.end(), PreciseSrcOffset.begin() + 1, PreciseSrcOffset.end());

      std::set<std::string> NewFuncs = mlta_infos[{Src.first, PreciseSrcOffset}];
      mlta_infos[{Dst.first, OffsetsDst}].insert(NewFuncs.begin(), NewFuncs.end());
    }
  }
}

void HPCFIBase::printCFIInfo(CallBase *I, std::vector<int> TargetCounts) {
  *OUT << "ind call: " << *I << "\n";
  DILocation *Loc = I->getDebugLoc();
  if (!Loc) {
    *OUT << "no debug info. compile with -g\n";
    return;
  }
  while (Loc) {
    StringRef File = Loc->getFilename();
    unsigned Line = Loc->getLine();
    unsigned Column = Loc->getColumn();
    *OUT << "filename: " << File.str() << " | " << Line << "," << Column << " |";
    Loc = Loc->getInlinedAt();
  }
  for (int TC : TargetCounts) {
    *OUT << " *** " << TC;
  }
  *OUT << "\n";
}

std::vector<CallBase *> HPCFIBase::getIndirectCalls() {
  std::vector<CallBase *> IndirectCalls;
  *OUT << "#functions: " << M->size() << "\n"; // does not include external functions
  int Count = 0;
  for (auto &F : *M) {
    if (F.hasAddressTaken())
      Count++;
  }
  *OUT << "#affunctions: " << Count << "\n";

  for (auto &F : *M) {
    for (auto &BB : F) {
      for (auto &I : BB) {
        if (CallBase *CB = dyn_cast<CallBase>(&I)) {
          if (!CB->isIndirectCall())
            continue;

          // ignore virtual calls
          if (CB->getMetadata("is_icall") == nullptr) {
            continue;
          }

          if (!ExclCallSites.empty()) {
            DILocation *Loc = CB->getDebugLoc();
            assert(Loc != nullptr && "debug info is needed to exclude "
                                     "functions from CFI instrumentation");
            std::pair<unsigned, std::string> P = {Loc->getLine(), Loc->getFilename().str()};
            if (ExclCallSites.find(P) != ExclCallSites.end()) {
              *OUT << "ignoring " << Loc->getFilename().str() << " | " << Loc->getLine() << "\n";
              continue;
            }
          }

          IndirectCalls.push_back(CB);
        }
      }
    }
  }
  return IndirectCalls;
}

void HPCFIBase::createCFIFailFunc() {
  FunctionCallee TrapCallee = M->getOrInsertFunction(
      "llvm.trap", FunctionType::get(Type::getVoidTy(M->getContext()), false));
  Function *Trap = dyn_cast<Function>(TrapCallee.getCallee());
  FunctionCallee FailFuncCallee = M->getOrInsertFunction(
      "cfi_fail", FunctionType::get(Type::getVoidTy(M->getContext()), false));
  CFIFailFunc = dyn_cast<Function>(FailFuncCallee.getCallee());
  BasicBlock *FailBB = BasicBlock::Create(M->getContext(), "f", CFIFailFunc);
  llvm::IRBuilder<> B(FailBB);
  B.CreateCall(Trap);
  B.CreateUnreachable();
}

std::set<Function *> HPCFIBase::findTargetsSVF(CallBase *CB) {
  std::set<Function *> Targets;
  SVF::CallICFGNode *CBN = Icfg->getCallICFGNode(CB);
  if (CallGraph->hasCallGraphEdge(CBN)) {
    for (auto I = CallGraph->getCallEdgeBegin(CBN), E = CallGraph->getCallEdgeEnd(CBN); I != E;
         ++I) {
      Function *Target = (*I)->getDstNode()->getFunction()->getLLVMFun();
      Targets.insert(Target);
    }
  }
  return Targets;
}

std::set<Function *> HPCFIBase::findTargetsType(CallBase *CB) {
  std::set<Function *> Targets;
  for (auto &F : *M) {
    if (!F.hasAddressTaken())
      continue;
    if (hasSameType(F.getFunctionType(), CB->getFunctionType()))
      Targets.insert(&F);
  }
  return Targets;
}

bool HPCFIBase::containsBlackHole(CallBase *CB) {
  SVF::NodeID CBNodeID = Pag->getValueNode(CB->getCalledOperand());
  const SVF::PointsTo &Pts = Ander->getPts(CBNodeID);
  for (SVF::NodeID P : Pts) {
    if (Pag->isBlkObj(P)) {
      return true;
    }
  }
  return false;
}

std::set<Function *> HPCFIBase::getTargets(CallBase *CB, bool Print) {
  std::set<Function *> TargetsSVF = findTargetsSVF(CB);
  std::set<Function *> TargetsSameType = findTargetsType(CB);
  std::set<Function *> *TargetsMLTA = findTargetsMLTA(CB);
  std::set<Function *> TargetsSVFType;
  std::set_intersection(TargetsSVF.begin(), TargetsSVF.end(), TargetsSameType.begin(),
                        TargetsSameType.end(),
                        std::inserter(TargetsSVFType, TargetsSVFType.begin()));

  std::set<Function *> TargetsUnion;
  TargetsUnion.insert(TargetsSVF.begin(), TargetsSVF.end());
  TargetsUnion.insert(TargetsSameType.begin(), TargetsSameType.end());
  if (TargetsMLTA != nullptr)
    TargetsUnion.insert(TargetsMLTA->begin(), TargetsMLTA->end());

  bool CanUseSvf = !containsBlackHole(CB);
  std::set<Function *> Targets = CanUseSvf ? TargetsSVFType : TargetsSameType;
  bool CanUseMlta = TargetsMLTA != nullptr;
  if (CanUseMlta) {
    std::set<Function *> NewTargets;
    std::set_intersection(Targets.begin(), Targets.end(), TargetsMLTA->begin(), TargetsMLTA->end(),
                          std::inserter(NewTargets, NewTargets.begin()));
    Targets = NewTargets;
  }

  if (Print) {
    std::vector<int> TCs = {
        static_cast<int>(Targets.size()), static_cast<int>(TargetsSVFType.size()),
        static_cast<int>(TargetsSVF.size()),
        static_cast<int>(CanUseMlta ? TargetsMLTA->size() : TargetsSameType.size()),
        static_cast<int>(TargetsSameType.size())};
    printCFIInfo(CB, TCs);

    if (CanUseMlta)
      *OUT << "MLTA target\n";

    if (containsBlackHole(CB))
      *OUT << "BLK!!\n";

    *OUT << "\n";
    for (Function *Target : TargetsUnion) {
      *OUT << "\t" << Target->getName();
      if (TargetsSVF.find(Target) != TargetsSVF.end())
        *OUT << " [SVF] ";
      if (TargetsSameType.find(Target) != TargetsSameType.end())
        *OUT << " [TYPE] ";
      if (TargetsMLTA != nullptr && TargetsMLTA->find(Target) != TargetsMLTA->end())
        *OUT << " [MLTA] ";
      *OUT << "\n";
    }
  }
  return Targets;
}

void HPCFIBase::setFuncnameToFuncs() {
  for (Function &F : *M) {
    if (!F.hasName())
      continue;
    std::string Name = F.getName().str();
    Name = Name.substr(0, Name.rfind("."));
    FuncnameToFunc[Name].insert(&F);
  }
}

std::set<Function *> *HPCFIBase::findTargetsMLTA(CallBase *CB) {
  std::set<Function *> *Targets = new std::set<Function *>();
  /*
  We have the following metadata:
  For indirect callsites:
  - which struct type gets loaded + which offset of that type

  For each module:
  - mapping from structtype, offset -> set<Function>
  */

  // metadata for indirect callsites look like this:
  // at CB: !mlta_info !63
  // !63 = !{!"struct.S", i32 1}
  MDNode *MltaInfo = CB->getMetadata(MLTA_INFO);
  if (MltaInfo == nullptr)
    return nullptr;
  std::string StructType = dyn_cast<MDString>(MltaInfo->getOperand(0))->getString().str();
  std::vector<int> Offsets;
  MDNode *OffsetsNode = dyn_cast<MDNode>(MltaInfo->getOperand(1));
  for (size_t I = 0; I < OffsetsNode->getNumOperands(); I++) {
    int Offset =
        dyn_cast<ConstantInt>(dyn_cast<ConstantAsMetadata>(OffsetsNode->getOperand(I))->getValue())
            ->getSExtValue();
    Offsets.push_back(Offset);
  }

  // start without offsets and add all offsets we found (stop if there is an
  // MLTA_INVALID)
  std::pair<std::string, std::vector<int>> P = {StructType, {}};
  for (unsigned I = 0; I < Offsets.size(); I++) {
    P.second.push_back(Offsets[I]);
    if (mlta_infos[P].size() == 0) {
      P.second.pop_back();
      break;
    }
    if (mlta_infos[P].find(MLTA_INVALID) != mlta_infos[P].end()) {
      P.second.pop_back();
      break;
    }
  }

  // stop if the first offset contains MLTA_INVALID
  if (P.second.size() == 0) {
    return nullptr;
  }

  for (std::string FuncName : mlta_infos[P]) {
    if (FuncnameToFunc.find(FuncName) != FuncnameToFunc.end()) {
      for (Function *F : FuncnameToFunc[FuncName])
        Targets->insert(F);
    } else {
      *OUT << "func not found: " << FuncName << "\n";
    }
  }

  return Targets;
}

bool HPCFIBase::isSameStructType(Type *S1, Type *S2) {
  while (!S1->isStructTy()) {
    if (!S1->isPointerTy() || !S2->isPointerTy())
      return false;
    S1 = dyn_cast<PointerType>(S1)->getElementType();
    S2 = dyn_cast<PointerType>(S2)->getElementType();
  }

  StructType *FS = dyn_cast<StructType>(S1);
  StructType *GS = dyn_cast<StructType>(S2);
  if (!FS || !GS)
    return false;

  if (!FS->hasName() || !GS->hasName())
    return false;

  std::string FNI = FS->getName().str();
  std::string GNI = GS->getName().str();
  if (std::count(FNI.begin(), FNI.end(), '.') >= 2) {
    size_t FirstPos = FNI.find(".");
    FNI = FNI.substr(0, FNI.find(".", FirstPos + 1));
  }
  if (std::count(GNI.begin(), GNI.end(), '.') >= 2) {
    size_t FirstPos = GNI.find(".");
    GNI = GNI.substr(0, GNI.find(".", FirstPos + 1));
  }

  return FNI == GNI;
}

bool HPCFIBase::hasSameType(FunctionType *FTy, FunctionType *GTy) {
  if (FTy->getNumParams() != GTy->getNumParams()) {
    return false;
  }
  if (FTy->isVarArg() != GTy->isVarArg()) {
    return false;
  }
  // args
  for (unsigned I = 0; I < FTy->getNumParams(); I++) {
    if (FTy->getParamType(I) != GTy->getParamType(I)) {
      if (!isSameStructType(FTy->getParamType(I), GTy->getParamType(I)))
        return false;
    }
  }
  // ret
  if (FTy->getReturnType() != GTy->getReturnType()) {
    if (!isSameStructType(FTy->getReturnType(), GTy->getReturnType()))
      return false;
  }
  return true;
}

} // namespace HPCFI