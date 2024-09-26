//===-- CmpCFIPass.cpp ----------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#include "../CmpCFIBase.h"

using namespace llvm;

namespace HPCFI {
struct CmpCFIPass : CmpCFIBase {
  static char ID;
  CmpCFIPass() : HPCFIBase(ID), CmpCFIBase(ID) {}

  StringRef getPassName() const override { return "cmpcfi"; }

  bool runOnModule(Module &M) override {
    HPCFIBase::runOnModule(M);
    std::vector<CallBase *> IndirectCalls = getIndirectCalls();
    for (CallBase *CB : IndirectCalls) {
      std::set<Function *> Targets = getTargets(CB, true);
      insertCmpCFICheck(CB, Targets);
    }

    return true;
  }
};
} // namespace HPCFI

char HPCFI::CmpCFIPass::ID = 77;
static RegisterPass<HPCFI::CmpCFIPass> X("cmpcfi", "CMP CFI");

static RegisterStandardPasses C(PassManagerBuilder::EP_FullLinkTimeOptimizationLast,
                                [](const PassManagerBuilder &Builder, legacy::PassManagerBase &PM) {
                                  PM.add(new HPCFI::CmpCFIPass());
                                });