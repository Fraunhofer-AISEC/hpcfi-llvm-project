//===-- CmpCFIBase.cpp  ---------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#include "CmpCFIBase.h"

using namespace llvm;

namespace HPCFI {
BasicBlock *CmpCFIBase::insertCmpCFICheck(CallBase *CB, std::set<Function *> Targets) {
  assert(CB->isIndirectCall());

  // Split block containing "Call %reg" into two blocks:
  // BBS contains every instruction before the call
  // BBC contains the call + every instruction after the call
  BasicBlock *BBC = CB->getParent()->splitBasicBlock(CB, "BBC");
  BasicBlock *BBS = BBC->getSinglePredecessor();
  BBS->getTerminator()->eraseFromParent();

  // Insert CFI checks between BB1 and BBC. Every check gives one block.
  llvm::IRBuilder<> B(BBS);
  BasicBlock *CurrBB = BasicBlock::Create(M->getContext(), "B", CB->getFunction(), BBC);
  BasicBlock *First = CurrBB;
  B.CreateBr(CurrBB);

  for (Function *Target : Targets) {
    BasicBlock *NextBB = BasicBlock::Create(M->getContext(), "B", CB->getFunction(), BBC);

    B.SetInsertPoint(CurrBB);
    Value *CastedTarget =
        B.CreateCast(Instruction::BitCast, Target, CB->getCalledOperand()->getType(), "Target");
    Value *IsValidPtr =
        B.CreateCmp(CmpInst::Predicate::ICMP_EQ, CastedTarget, CB->getCalledOperand(), "Check");
    B.CreateCondBr(IsValidPtr, BBC, NextBB);

    CurrBB = NextBB;
  }

  // Create CFI Fail Block
  B.SetInsertPoint(CurrBB);
  B.CreateCall(CFIFailFunc);
  B.CreateUnreachable();
  return First;
}
} // namespace HPCFI