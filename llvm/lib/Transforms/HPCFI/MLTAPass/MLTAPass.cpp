//===-- MLTAPass.cpp  -----------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"

#include "MLTA.h"

using namespace llvm;

/*
Add metadata to indirect callsites:
- which struct type gets loaded + which offset of that type

Add metadata to module:
- mapping from structtype, offset -> set<Function>
*/
namespace HPCFI {
struct MLTAPass : public ModulePass {
  static char ID;
  MLTAPass() : ModulePass(ID) {}

  StringRef getPassName() const override { return "mlta"; }

  bool runOnModule(Module &M) override {
    MLTA *FA = new MLTA(M);

    int Id = 0;
    for (auto FieldInfo : FA->field_infos) {
      std::string StructType = FieldInfo.first.first;
      std::vector<int> Offsets = FieldInfo.first.second;
      std::set<std::string> Targets = FieldInfo.second;

      MDString *StructTypeMd = MDString::get(M.getContext(), StructType);
      std::vector<Metadata *> OffsetsMd;
      std::vector<Metadata *> TargetsMd;
      for (int Offset : Offsets) {
        OffsetsMd.push_back(
            ConstantAsMetadata::get(ConstantInt::get(Type::getInt32Ty(M.getContext()), Offset)));
      }
      for (std::string Target : Targets) {
        TargetsMd.push_back(MDString::get(M.getContext(), Target));
      }
      MDNode *TargetsMdNode = MDNode::get(M.getContext(), TargetsMd);
      MDNode *OffsetsMdNode = MDNode::get(M.getContext(), OffsetsMd);

      Metadata *StructMetadata[3] = {StructTypeMd, OffsetsMdNode, TargetsMdNode};
      MDNode *Metadata = MDNode::get(M.getContext(), StructMetadata);
      M.addModuleFlag(llvm::Module::ModFlagBehavior::Warning,
                      std::string(MLTA_STRUCT_INFOS) + M.getName().str() + std::to_string(Id++) +
                          std::to_string(rand()),
                      Metadata);
    }

    Id = 0;
    for (auto GepInFptrarrargs : FA->geps_in_fptrarrargs) {
      std::string StructType = GepInFptrarrargs.first.first;
      int Offset = GepInFptrarrargs.first.second;
      std::set<std::pair<std::string, unsigned>> FunctionArgnos = GepInFptrarrargs.second;

      MDString *StructTypeMd = MDString::get(M.getContext(), StructType);
      Metadata *OffsetMd =
          ConstantAsMetadata::get(ConstantInt::get(Type::getInt32Ty(M.getContext()), Offset));
      std::vector<Metadata *> FunctionsAndArgnos;
      for (auto FunctionArgno : FunctionArgnos) {
        FunctionsAndArgnos.push_back(MDString::get(M.getContext(), FunctionArgno.first));
        FunctionsAndArgnos.push_back(ConstantAsMetadata::get(
            ConstantInt::get(Type::getInt32Ty(M.getContext()), FunctionArgno.second)));
      }
      MDNode *FunctionsAndArgnosMdNode = MDNode::get(M.getContext(), FunctionsAndArgnos);
      Metadata *MD[3] = {StructTypeMd, OffsetMd, FunctionsAndArgnosMdNode};
      MDNode *Metadata = MDNode::get(M.getContext(), MD);
      M.addModuleFlag(llvm::Module::ModFlagBehavior::Warning,
                      std::string(MLTA_GEPS_IN_FPTRARRARGS) + M.getName().str() +
                          std::to_string(Id++) + std::to_string(rand()),
                      Metadata);
    }

    Id = 0;
    for (auto FptrarrArgInit : FA->fptrarr_arg_inits) {
      std::string Function = FptrarrArgInit.first.first;
      int Argno = FptrarrArgInit.first.second;
      std::map<std::vector<int>, std::set<std::string>> OffsetsAndTargets = FptrarrArgInit.second;

      MDString *FunctionNameMd = MDString::get(M.getContext(), Function);
      Metadata *ArgnoMd =
          ConstantAsMetadata::get(ConstantInt::get(Type::getInt32Ty(M.getContext()), Argno));
      std::vector<Metadata *> OffsetsToTargets;
      for (auto Ot : OffsetsAndTargets) {
        std::vector<int> Offsets = Ot.first;
        std::set<std::string> Targets = Ot.second;
        std::vector<Metadata *> OffsetsMd;
        std::vector<Metadata *> TargetsMd;
        for (int Offset : Offsets) {
          OffsetsMd.push_back(
              ConstantAsMetadata::get(ConstantInt::get(Type::getInt32Ty(M.getContext()), Offset)));
        }
        for (std::string Target : Targets) {
          TargetsMd.push_back(MDString::get(M.getContext(), Target));
        }
        OffsetsToTargets.push_back(MDNode::get(M.getContext(), OffsetsMd));
        OffsetsToTargets.push_back(MDNode::get(M.getContext(), TargetsMd));
      }
      MDNode *OffsetsAndTargetsMdNode = MDNode::get(M.getContext(), OffsetsToTargets);
      Metadata *MD[3] = {FunctionNameMd, ArgnoMd, OffsetsAndTargetsMdNode};
      MDNode *Metadata = MDNode::get(M.getContext(), MD);
      M.addModuleFlag(llvm::Module::ModFlagBehavior::Warning,
                      std::string(MLTA_FPTRARR_ARG_INITS) + M.getName().str() +
                          std::to_string(Id++) + std::to_string(rand()),
                      Metadata);
    }

    Id = 0;
    for (auto MemcpyAlias : FA->memcpy_aliases) {
      std::string DstStructName = MemcpyAlias.first.first;
      int DstOffset = MemcpyAlias.first.second;
      std::string SrcStructName = MemcpyAlias.second.first;
      int SrcOffset = MemcpyAlias.second.second;
      MDString *DstStructNameMd = MDString::get(M.getContext(), DstStructName);
      Metadata *DstOffsetMd =
          ConstantAsMetadata::get(ConstantInt::get(Type::getInt32Ty(M.getContext()), DstOffset));
      MDString *SrcStructNameMd = MDString::get(M.getContext(), SrcStructName);
      Metadata *SrcOffsetMd =
          ConstantAsMetadata::get(ConstantInt::get(Type::getInt32Ty(M.getContext()), SrcOffset));
      Metadata *MD[4] = {DstStructNameMd, DstOffsetMd, SrcStructNameMd, SrcOffsetMd};
      MDNode *Metadata = MDNode::get(M.getContext(), MD);
      M.addModuleFlag(llvm::Module::ModFlagBehavior::Warning,
                      std::string(MLTA_MEMCPY_ALIASES) + M.getName().str() + std::to_string(Id++) +
                          std::to_string(rand()),
                      Metadata);
    }

    // create metadata for indirect calls
    for (Function &F : M) {
      for (BasicBlock &BB : F) {
        for (Instruction &I : BB) {
          if (CallBase *CB = dyn_cast<CallBase>(&I)) {
            if (!CB->isIndirectCall())
              continue;

            std::pair<std::string, std::vector<int>> StructTypeAndOffsets =
                FA->getStructTypeAndOffset(CB);
            if (StructTypeAndOffsets.second.size() == 0)
              continue;

            MDString *StructType = MDString::get(M.getContext(), StructTypeAndOffsets.first);
            std::vector<Metadata *> OffsetsMd;
            for (int Offset : StructTypeAndOffsets.second) {
              OffsetsMd.push_back(ConstantAsMetadata::get(
                  ConstantInt::get(Type::getInt32Ty(M.getContext()), Offset)));
            }
            MDNode *OffsetsMdNode = MDNode::get(M.getContext(), OffsetsMd);

            Metadata *CBMetadata[2] = {StructType, OffsetsMdNode};
            MDNode *Metadata = MDNode::get(M.getContext(), CBMetadata);
            CB->setMetadata(MLTA_INFO, Metadata);
          }
        }
      }
    }
    return false;
  }
};
} // namespace HPCFI

char HPCFI::MLTAPass::ID = 2;
static RegisterPass<HPCFI::MLTAPass> A("mltapass", "MLTA Pass");

static RegisterStandardPasses D(PassManagerBuilder::EP_ModuleOptimizerEarly,
                                [](const PassManagerBuilder &Builder, legacy::PassManagerBase &PM) {
                                  PM.add(new HPCFI::MLTAPass());
                                });
