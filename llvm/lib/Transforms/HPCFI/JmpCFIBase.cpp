//===-- JmpCFIBase.cpp  ---------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#include "JmpCFIBase.h"

#include "llvm/IR/Constants.h"
#include "llvm/IR/InlineAsm.h"
#include "llvm/IR/Intrinsics.h"

#include <fstream>

using namespace llvm;

namespace HPCFI {

Function *JmpCFIBase::createJumptableAndReplaceFptrs(FuncSet &Targets, unsigned jt_count) {
  // Create Jumptable
  std::stringstream ss;
  ss << "jumptable" << jt_count;
  FunctionCallee JTCallee =
      M->getOrInsertFunction(ss.str(), FunctionType::get(Type::getVoidTy(M->getContext()), false));
  Function *JT = dyn_cast<Function>(JTCallee.getCallee());
  BasicBlock *BB = BasicBlock::Create(M->getContext(), "f", JT);
  llvm::IRBuilder<> JTBuilder(BB);

  // For each target, add the function pointer to the jumptable.
  // Next, replace all function pointers with the corresponding jump table
  // entry.
  int nr_jt_entries = 0;
  for (Function *Fptr : Targets) {
    // Create inline assembly instruction: jmp Fptr
    std::string JmpInst =
        std::string("jmp ") + std::string(Fptr->getName()) + std::string("\n.align 8");
    InlineAsm *IA = InlineAsm::get(FunctionType::get(Type::getVoidTy(M->getContext()), false),
                                   JmpInst, "", true, false);
    JTBuilder.CreateCall(IA);

    // Replace function pointer with jumptable entry
    Constant *JTOffset =
        ConstantInt::get(IntegerType::getInt64Ty(M->getContext()), 8 * nr_jt_entries++, true);
    Constant *JTInt = ConstantExpr::getPtrToInt(JT, IntegerType::getInt64Ty(M->getContext()));
    Constant *TargetInt = ConstantExpr::getAdd(JTInt, JTOffset);

    // replace the function pointer with the jumptable entry
    Fptr->replaceAllUsesWith(ConstantExpr::getIntToPtr(TargetInt, Fptr->getType()));
  }

  JTBuilder.CreateUnreachable();
  return JT;
}

void JmpCFIBase::insertJtCFICheck(CallBase *CB, Function *JT, BasicBlock *BBStart,
                                  BasicBlock *BBCont, BasicBlock *FailBB, unsigned nr_jt_entries) {
  assert(CB->isIndirectCall());

  // Check if pointer is in Jumptable:
  // check:
  //   offset = pointer - jumptable
  //   offset_shr = shr offset, 3
  //   offset_shl = shl offset, 61
  //   new_offset = offset_shr or offset_shl
  //   is_valid_ptr = cmp new_offset, nr_jt_entries - 1
  //   jle cont
  //   FailBB
  // cont:
  //   call pointer

  llvm::IRBuilder<> B(BBStart);
  Value *PointerInt = B.CreatePtrToInt(CB->getCalledOperand(),
                                       IntegerType::getInt64Ty(M->getContext()), "pointer_int");
  Constant *JTInt = ConstantExpr::getPtrToInt(JT, IntegerType::getInt64Ty(M->getContext()));
  Value *Offset = B.CreateSub(PointerInt, JTInt, "offset");
  Value *OffsetShr = B.CreateLShr(
      Offset, ConstantInt::get(IntegerType::getInt64Ty(M->getContext()), 3, true), "offset_shr");
  Value *OffsetShl =
      B.CreateShl(Offset,
                  ConstantInt::get(IntegerType::getInt64Ty(M->getContext()),
                                   M->getDataLayout().getPointerSizeInBits() - 3, true),
                  "offset_shl");
  Value *NewOffset = B.CreateOr(OffsetShr, OffsetShl, "new_offset");
  Value *IsValidPtr = B.CreateICmpULE(
      NewOffset, ConstantInt::get(IntegerType::getInt64Ty(M->getContext()), nr_jt_entries - 1),
      "is_valid_ptr");
  B.CreateCondBr(IsValidPtr, BBCont, FailBB);
}
} // namespace HPCFI