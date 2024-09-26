//===-- MLTA.cpp  ---------------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#include "llvm/Demangle/Demangle.h"
#include "llvm/IR/DebugInfoMetadata.h"

#include <queue>

#include "SVF-FE/GEPTypeBridgeIterator.h"

#include "MLTA.h"

using namespace llvm;

namespace HPCFI {

bool isFunctionPointerType(const Type *Ty) {
  return (Ty->isPointerTy() && Ty->getPointerElementType()->isFunctionTy());
}

bool isArrayOfFunctionPointersType(const Type *Ty) {
  const ArrayType *ArrTy = dyn_cast<ArrayType>(Ty);
  while (ArrTy != nullptr) {
    Type *ElemTy = ArrTy->getElementType();
    if (isFunctionPointerType(ElemTy))
      return true;
    ArrTy = dyn_cast<ArrayType>(ElemTy);
  }
  return false;
}

bool containsFptrType(const Type *Ty) {
  if (Ty->isFunctionTy())
    return true;
  if (const PointerType *PT = dyn_cast<PointerType>(Ty))
    return containsFptrType(PT->getElementType());
  if (const ArrayType *AT = dyn_cast<ArrayType>(Ty))
    return containsFptrType(AT->getElementType());
  return false;
}

bool isPointerToArrayOfFunctionPointersType(const Type *Ty) {
  return Ty->isPointerTy() && isArrayOfFunctionPointersType(Ty->getPointerElementType());
}

bool isArrayPointerType(const Type *Ty) {
  return (Ty->isPointerTy() && Ty->getPointerElementType()->isArrayTy());
}

std::vector<std::pair<const Type *, const Value *>> getRightToLeftGEPTypes(GEPOperator *GEP) {
  std::vector<std::pair<const Type *, const Value *>> Types =
      std::vector<std::pair<const Type *, const Value *>>();
  for (bridge_gep_iterator Gi = bridge_gep_begin(*GEP), Ge = bridge_gep_end(*GEP); Gi != Ge; ++Gi) {
    Types.insert(Types.begin(), {*Gi, Gi.getOperand()});
  }
  return Types;
}

// checks if the rightmost type of the GEP is a struct type
bool gepFptrOnStructType(GEPOperator *GEP) {
  auto Types = getRightToLeftGEPTypes(GEP);
  for (auto Type : Types) {
    if (Type.first->isStructTy()) {
      return true;
    }
    if (isArrayOfFunctionPointersType(Type.first))
      continue;
    return false;
  }
  return false;
}

std::string getStructName(const StructType *ST) {
  std::string S = ST->getName().str();
  size_t FirstDotIndex = S.find('.');
  if (FirstDotIndex != std::string::npos) {
    size_t SecondDotIndex = S.find('.', FirstDotIndex + 1);
    if (SecondDotIndex != std::string::npos) {
      S = S.substr(0, SecondDotIndex);
    }
  }
  return S;
}

// get struct type and field offset to function pointer or array of function pointers
// this corresponds to the rightmost struct type and corresponding offset
std::pair<std::string, int> getGEPStructTypeOffset(GEPOperator *GEP) {
  // considering GEP chains is not necessary if pass runs early enough
  auto Types = getRightToLeftGEPTypes(GEP);

  for (auto Type : Types) {
    if (Type.first->isStructTy()) {
      const StructType *ST = dyn_cast<StructType>(Type.first);
      const ConstantInt *OP = dyn_cast<ConstantInt>(Type.second);
      assert(ST != nullptr);
      assert(OP != nullptr);
      return {ST->hasName() ? getStructName(ST) : "", (int)OP->getSExtValue()};
    }
    if (isArrayOfFunctionPointersType(Type.first))
      continue;
    assert(false);
  }
  assert(false);
  return {"", -1};
}

MLTA::MLTA(Module &M) {
  setFunctionPointerArrayArgs(M);
  setStores(M);
  traverseGlobals(M);

  for (auto &MemcpyDest : memcpy_dests) {
    for (auto &Ms : memcpy_sources[MemcpyDest.first]) {
      for (auto &Md : MemcpyDest.second) {
        memcpy_aliases.insert({Md, Ms});
      }
    }
  }
}

int getArgNo(CallBase *CB, Value *V) {
  Function *F = CB->getCalledFunction();
  for (unsigned I = 0; I < CB->arg_size(); I++) {
    if (CB->getArgOperand(I) == V && I < F->arg_size()) {
      return I;
    }
  }
  return -1;
}

std::set<std::string>
MLTA::followGEPToStore(Value *GEP, std::set<std::pair<std::string, unsigned>> *FptrArrayArgs,
                       std::set<CallBase *> *MemcpyDests, std::set<CallBase *> *MemcpySources) {
  std::set<std::string> Ret;
  // for function pointer arrays, multiple GEPs may exist (e.g., init functions)
  // for all GEPs:
  // - users must either be GEPs
  // - or user is load (ignore) or store (add function pointer)
  // - if anything else: add to unknown_fields
  std::queue<Value *> Worklist;
  Worklist.push(GEP);
  while (!Worklist.empty()) {
    Value *V = Worklist.front();
    Worklist.pop();
    for (User *U : V->users()) {
      if (isa<GEPOperator>(U) || isa<BitCastOperator>(U)) {
        Worklist.push(U);
      } else if (SelectInst *SI = dyn_cast<SelectInst>(U)) {
        Worklist.push(SI);
      } else if (isa<LoadInst>(U)) {
        continue;
      } else if (StoreInst *SI = dyn_cast<StoreInst>(U)) {
        Value *StoredValue = SI->getOperand(0);
        if (Function *F = dyn_cast<Function>(StoredValue->stripPointerCasts())) {
          Ret.insert(F->getName().str());
          continue;
        }
        Ret.insert(MLTA_INVALID);
      } else if (CallBase *CB = dyn_cast<CallBase>(U)) {
        if (FptrArrayArgs != nullptr && containsFptrType(V->getType())) {
          if (Function *F = CB->getCalledFunction()) {
            int Arg = getArgNo(CB, V);
            if (Arg == -1)
              Ret.insert(MLTA_INVALID);
            else {
              FptrArrayArgs->insert({F->getName().str(), Arg});
            }
          } else
            Ret.insert(MLTA_INVALID);
        }
        if (Function *F = CB->getCalledFunction()) {
          if (F->getName().contains("memcpy") && MemcpyDests != nullptr &&
              MemcpySources != nullptr) {
            if (CB->getArgOperand(0) == V) {
              MemcpyDests->insert(CB);
            } else {
              MemcpySources->insert(CB);
            }
          }
        }
      } else {
        Ret.insert(MLTA_INVALID);
      }
    }
  }
  return Ret;
}

std::map<std::vector<int>, std::set<std::string>>
MLTA::traverseGeps(Value *GEP, std::set<std::pair<std::string, unsigned>> *FptrArrayArgs,
                   std::set<CallBase *> *MemcpyDests, std::set<CallBase *> *MemcpySources) {
  std::map<std::vector<int>, std::set<std::string>> Ret;
  // we rely on the fact that the LLVM IR always splits GEPs for each index, e.g., for:
  //    s.fs[2][3] = funcA;
  // we get
  //    %fs = getelementptr inbounds %struct.S, %struct.S* %s, i32 0, i32 0, !dbg !29
  //    %arrayidx = getelementptr inbounds [5 x [7 x i32 (i32)*]], [5 x [7 x i32 (i32)*]]* %fs, i64
  //    0, i64 2, !dbg !30 %arrayidx1 = getelementptr inbounds [7 x i32 (i32)*], [7 x i32 (i32)*]*
  //    %arrayidx, i64 0, i64 3, !dbg !30

  // for array offset 0 there may not be a GEP
  for (User *U : GEP->users()) {
    if (StoreInst *SI = dyn_cast<StoreInst>(U)) {
      Value *StoredValue = SI->getOperand(0);
      if (Function *F = dyn_cast<Function>(StoredValue->stripPointerCasts())) {
        std::vector<int> Vec0 = {0};
        Ret[Vec0].insert(F->getName().str());
      }
    }
  }

  std::queue<std::pair<Value *, std::vector<int>>> Worklist;
  Worklist.push({GEP, {}});
  while (!Worklist.empty()) {
    auto V = Worklist.front();
    Worklist.pop();
    Value *GEP = V.first;
    std::vector<int> Offsets = V.second;
    std::set<std::string> Funcs = followGEPToStore(GEP, FptrArrayArgs, MemcpyDests, MemcpySources);
    Ret[Offsets] = Funcs;
    bool IsStore = Funcs.size() > 0;

    for (User *U : GEP->users()) {
      if (GEPOperator *NewGep = dyn_cast<GEPOperator>(U)) {
        // assume that GEPs only have two indices: 0 and the actual one (for arrays sometimes only
        // one index)
        if (ConstantInt *CI =
                dyn_cast<ConstantInt>(NewGep->getOperand(NewGep->getNumOperands() - 1))) {
          std::vector<int> NewOffsets = {(int)CI->getSExtValue()};
          NewOffsets.insert(NewOffsets.begin(), Offsets.begin(), Offsets.end());
          Worklist.push({NewGep, NewOffsets});
        } else if (IsStore) {
          // if the index is not constant (AND we have a store):
          // we need to add the value to all offsets of the array...
          // e.g. for s.a[i] = funcA
          // func A must be added to s.a, but also to s.a[1]
          Ret[Offsets].insert(MLTA_INVALID);
        }
      }
    }
  }
  return Ret;
}

void MLTA::setStores(Module &M) {
  for (Function &F : M) {
    for (BasicBlock &BB : F) {
      for (Instruction &I : BB) {
        Value *IV = &I;
        // e.g.stores to globals
        if (StoreInst *SI = dyn_cast<StoreInst>(IV)) {
          IV = SI->getOperand(1);
        }
        if (GEPOperator *GEP = dyn_cast<GEPOperator>(IV)) {
          const Type *Type = GEP->getResultElementType();
          // we only care about GEPs indexing:
          //  - a function pointer
          //  - an array of function pointers
          // if the type before the return type is not a struct, continue
          // if type is not a function pointer, continue
          if ((!isFunctionPointerType(Type) && !isArrayOfFunctionPointersType(Type)) ||
              !gepFptrOnStructType(GEP)) {
            if (gepFptrOnStructType(GEP)) {
              std::pair<std::string, int> P = getGEPStructTypeOffset(GEP);
              field_infos[{P.first, {P.second}}].insert(MLTA_INVALID);
            }
            continue;
          }

          std::pair<std::string, int> P = getGEPStructTypeOffset(GEP);
          std::set<std::pair<std::string, unsigned>> FptrArrayArgs;
          std::set<CallBase *> MemcpyDests;
          std::set<CallBase *> MemcpySources;
          std::map<std::vector<int>, std::set<std::string>> Res =
              traverseGeps(GEP, &FptrArrayArgs, &MemcpyDests, &MemcpySources);
          for (auto &Entry : Res) {
            std::vector<int> Offsets = Entry.first;
            Offsets.insert(Offsets.begin(), P.second);
            field_infos[{P.first, Offsets}].insert(Entry.second.begin(), Entry.second.end());
          }
          geps_in_fptrarrargs[P].insert(FptrArrayArgs.begin(), FptrArrayArgs.end());
          for (CallBase *CB : MemcpyDests) {
            this->memcpy_dests[CB].insert(P);
          }
          for (CallBase *CB : MemcpySources) {
            this->memcpy_sources[CB].insert(P);
          }
        }
      }
    }
  }
}

void MLTA::traverseGlobal(GlobalVariable *GV, Constant *C, ConstantStruct *LastStruct,
                          int LastOffset) {
  if (C->getType()->isSingleValueType()) {
    // ignore
  } else if (isa<ConstantArray>(C)) {
    for (int I = 0, E = C->getNumOperands(); I != E; I++) {
      Constant *OP = dyn_cast<Constant>(C->getOperand(I));
      if (LastStruct != nullptr) {
        if (Function *F = dyn_cast<Function>(OP->stripPointerCasts())) {
          field_infos[{LastStruct->getType()->hasName() ? getStructName(LastStruct->getType()) : "",
                       {LastOffset}}]
              .insert(F->getName().str());
        }
      }
      traverseGlobal(GV, OP, LastStruct, LastOffset);
    }
  } else if (ConstantStruct *CS = dyn_cast<ConstantStruct>(C)) {
    for (int I = 0, E = CS->getNumOperands(); I != E; I++) {
      Constant *OP = dyn_cast<Constant>(CS->getOperand(I));
      if (Function *F = dyn_cast<Function>(OP->stripPointerCasts())) {
        if (CS->getType()->hasName()) {
          field_infos[{getStructName(CS->getType()), {I}}].insert(F->getName().str());
        }
        // llvm sometimes removes the explicit type for globals. get explicit type from metadata
        // instead.
        else if (LastStruct == nullptr) {
          MDNode *MD = GV->getMetadata(0);
          if (MD != nullptr) {
            DIGlobalVariableExpression *GVE = dyn_cast<DIGlobalVariableExpression>(MD);
            DIGlobalVariable *GV = GVE->getVariable();
            DIType *T = GV->getType();
            while (isa<DIDerivedType>(T)) { // follow typedefs
              T = dyn_cast<DIDerivedType>(T)->getBaseType();
            }
            if (DICompositeType *CT = dyn_cast<DICompositeType>(T)) {
              // example C: !107 = distinct !DICompositeType(tag: DW_TAG_structure_type, name:
              // "mgvtbl", file: !101, line: 11, size: 512, elements: !108) example C++: !1027 =
              // distinct !DICompositeType(tag: DW_TAG_structure_type, scope: !968, file: !18, line:
              // 1344, size: 256, flags: DIFlagTypePassByValue, elements: !1028, identifier:
              // "_ZTSN3pov14Pigment_StructUt_Ut_E")
              std::string Name = CT->getName().str();
              if (!CT->getName().startswith("struct."))
                Name = "struct." + Name;
              std::string Demangled = demangle(CT->getIdentifier().str());
              if (!Demangled.empty()) {
                // demangled returns "struct.typeinfo name for pov::Method_Struct" for some reason
                size_t LastSpacePos = Demangled.rfind(' ');
                if (LastSpacePos != std::string::npos) {
                  Demangled = Demangled.substr(LastSpacePos + 1);
                }
                Name = "struct." + Demangled;
              }

              field_infos[{Name, {I}}].insert(F->getName().str());
            } else {
              assert(false && "not a DICompositeType");
            }
          }
        }
      }
      traverseGlobal(GV, OP, CS, I);
    }
  } else if (isa<ConstantData>(C)) {
    // ignore
  } else {
    assert(false);
  }
}

void MLTA::setFunctionPointerArrayArgs(Module &M) {
  for (Function &F : M) {
    for (unsigned I = 0; I < F.arg_size(); I++) {
      Argument *A = F.getArg(I);
      if (!containsFptrType(A->getType()))
        continue;

      std::map<std::vector<int>, std::set<std::string>> Res =
          traverseGeps(A, nullptr, nullptr, nullptr);
      for (auto &R : Res) {
        fptrarr_arg_inits[{F.getName().str(), I}][R.first].insert(R.second.begin(), R.second.end());
      }
    }
  }
}

void MLTA::traverseGlobals(Module &M) {
  for (Module::global_iterator I = M.global_begin(), E = M.global_end(); I != E; ++I) {
    GlobalVariable *GV = &*I;
    if (GV->hasInitializer()) {
      traverseGlobal(GV, GV->getInitializer(), nullptr, 0);
    }
  }
}

std::pair<std::string, std::vector<int>> MLTA::getStructTypeAndOffset(const CallBase *CB) {
  Value *CalledOperand = CB->getCalledOperand();

  // find load
  LoadInst *LI = dyn_cast<LoadInst>(CalledOperand);
  if (LI == nullptr) {
    return {"", {}};
  }

  // find GEP
  Value *LoadedValue = LI->getOperand(0);
  while (BitCastOperator *BC = dyn_cast<BitCastOperator>(LoadedValue)) {
    LoadedValue = BC->getOperand(0);
  }

  GEPOperator *GEP = dyn_cast<GEPOperator>(LoadedValue);
  if (GEP == nullptr) {
    return {"", {}};
  }

  bool Fail = false;
  std::vector<int> Offsets;
  while (!gepFptrOnStructType(GEP)) {
    if (ConstantInt *CI = dyn_cast<ConstantInt>(GEP->getOperand(GEP->getNumOperands() - 1))) {
      Offsets.insert(Offsets.begin(), CI->getSExtValue());
    } else {
      Fail = true;
    }
    GEP = dyn_cast<GEPOperator>(GEP->getOperand(0));

    if (GEP == nullptr) {
      return {"", {}};
    }
  }
  std::pair<std::string, int> POld = getGEPStructTypeOffset(GEP);
  std::pair<std::string, std::vector<int>> P = {POld.first, {POld.second}};
  if (!Fail)
    P.second.insert(P.second.end(), Offsets.begin(), Offsets.end());
  return P;
}

} // namespace HPCFI