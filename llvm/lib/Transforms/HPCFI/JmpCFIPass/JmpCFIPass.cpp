//===-- JmpCFIPass.cpp  ---------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#include "../CmpCFIBase.h"
#include "../JmpCFIBase.h"

using namespace llvm;

namespace HPCFI {
struct JmpCFIPass : JmpCFIBase, CmpCFIBase {
  static char ID;

  unsigned max_cmp_targets = 3;

  // for all callsites, keep track of all possible targets
  std::map<CallBase *, FuncSet> CallsiteToTargets;
  std::map<Function *, std::set<CallBase *>> TargetToCallsites;

  JmpCFIPass() : HPCFIBase(ID), JmpCFIBase(ID), CmpCFIBase(ID) {}

  StringRef getPassName() const override { return "jmpcfipass"; }

  bool runOnModule(Module &M) override {
    HPCFIBase::runOnModule(M);

    std::vector<CallBase *> IndirectCalls = getIndirectCalls();

    for (CallBase *CB : IndirectCalls) {
      std::set<Function *> Targets = getTargets(CB);
      CallsiteToTargets[CB].insert(Targets.begin(), Targets.end());
      for (Function *T : Targets) {
        TargetToCallsites[T].insert(CB);
      }
    }

    // Merge: merging is necessary so that we can have a unique jumptable entry for all function
    // pointers first get all call sites that should not be instrumented using a jumptable
    std::map<CallBase *, FuncSet> CmpCallsiteToTargets;
    for (auto CT : CallsiteToTargets) {
      CallBase *CB = CT.first;
      FuncSet Targets = CT.second;
      if (Targets.size() <= max_cmp_targets) {
        CmpCallsiteToTargets[CB] = Targets;
      }
    }
    for (auto CT : CmpCallsiteToTargets) {
      CallsiteToTargets.erase(CT.first);
    }

    // merge callsites containing the same functions
    std::vector<std::pair<std::set<CallBase *>, FuncSet>> MergedJmpCallsitesToTargets;
    while (!CallsiteToTargets.empty()) {
      auto CT = CallsiteToTargets.begin();
      FuncSet Targets = CT->second;
      std::set<CallBase *> Callsites;
      Callsites.insert(CT->first);
      MergeCallsites(Callsites, Targets);
      MergedJmpCallsitesToTargets.push_back({Callsites, Targets});
      for (CallBase *CB : Callsites) {
        CallsiteToTargets.erase(CB);
      }
    }

    // Create CMP checks
    for (auto &CSToTargets : CmpCallsiteToTargets) {
      CallBase *CB = CSToTargets.first;
      FuncSet Targets = CSToTargets.second;

      insertCmpCFICheck(CB, Targets);
      std::vector<int> TCs = {static_cast<int>(Targets.size()), -1, -1, -1, -1};
      printCFIInfo(CB, TCs);
    }

    //  Create Jumptable + insert CFI checks
    unsigned jt_count = 0;
    for (auto &CSToTarget : MergedJmpCallsitesToTargets) {
      std::set<CallBase *> Callsites = CSToTarget.first;
      FuncSet Targets = CSToTarget.second;

      // Create Jumptable
      unsigned nr_jumptable_entries = Targets.size();
      Function *JT = createJumptableAndReplaceFptrs(Targets, jt_count++);

      // Insert CFI checks at all callsites.
      // Every Callsite uses the same jumptable.
      for (CallBase *CB : Callsites) {
        // create cfi fail block
        BasicBlock *BBCFIFail = BasicBlock::Create(M.getContext(), "cfi_fail", CB->getFunction());
        llvm::IRBuilder<> B(BBCFIFail);
        B.CreateCall(CFIFailFunc);
        B.CreateUnreachable();

        // create cfi cont block
        BasicBlock *BBCont = CB->getParent()->splitBasicBlock(CB, "bb_cont");
        BasicBlock *BBStart = BBCont->getSinglePredecessor();
        BBStart->getTerminator()->eraseFromParent();

        insertJtCFICheck(CB, JT, BBStart, BBCont, BBCFIFail, nr_jumptable_entries);
        std::vector<int> TCs = {static_cast<int>(nr_jumptable_entries), -1, -1, -1, -1};
        printCFIInfo(CB, TCs);
      }
    }

    return true;
  }

  void MergeCallsites(std::set<CallBase *> &Callsites, FuncSet &Targets) {
    FuncSet TargetStack = Targets;

    while (!TargetStack.empty()) {
      Function *Target = *TargetStack.begin();
      TargetStack.erase(Target);

      for (CallBase *CB : TargetToCallsites[Target]) {
        // skip if CMP
        if (CallsiteToTargets.find(CB) == CallsiteToTargets.end())
          continue;

        if (Callsites.find(CB) != Callsites.end())
          continue;

        Callsites.insert(CB);
        Targets.insert(CallsiteToTargets[CB].begin(), CallsiteToTargets[CB].end());
        TargetStack.insert(CallsiteToTargets[CB].begin(), CallsiteToTargets[CB].end());
      }
    }
  }
};

} // namespace HPCFI

char HPCFI::JmpCFIPass::ID = 99;
static RegisterPass<HPCFI::JmpCFIPass> A("jtcfi", "Jumptable CFI Pass");

static RegisterStandardPasses D(PassManagerBuilder::EP_FullLinkTimeOptimizationLast,
                                [](const PassManagerBuilder &Builder, legacy::PassManagerBase &PM) {
                                  PM.add(new HPCFI::JmpCFIPass());
                                });