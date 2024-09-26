//===-- HPCFIBase.h  ------------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#ifndef HPCFIBASE_H_
#define HPCFIBASE_H_

#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"

#include "SVF-FE/SVFIRBuilder.h"
#include "WPA/Andersen.h"

using namespace llvm;

namespace HPCFI {
class HPCFIBase : public ModulePass {
private:
  // A set of call sites (line + filename pairs) that will not be instrumented.
  std::set<std::pair<unsigned, std::string>> ExclCallSites;

  // mapping from function name to functions (funcA.1 and funcA.2 should be the
  // same function)
  std::map<std::string, std::set<Function *>> FuncnameToFunc;

  // Parse ignored functions file
  void setExclCallSites();

  void setFuncnameToFuncs();

  // Create function to be called on CFI violation
  void createCFIFailFunc();

  // combine MLTA infos from metadata
  void updateMLTAInfos();

  // some struct types are changed by llvm: e.g. %struct.A -> %struct.A.1115
  bool isSameStructType(Type *, Type *);

protected:
  Module *M;
  raw_fd_ostream *OUT;

  SVF::PAG *Pag;
  SVF::Andersen *Ander;
  SVF::SVFG *Svfg;
  SVF::ICFG *Icfg;
  SVF::PTACallGraph *CallGraph;

  Function *CFIFailFunc;

  HPCFIBase(char ID) : ModulePass(ID) {}

  // MLTA infos: mapping from struct_name and offsets to functions
  std::map<std::pair<std::string, std::vector<int>>, std::set<std::string>> mlta_infos;

  bool runOnModule(Module &M) override;

  // Print debug info of instruction + number of targets
  void printCFIInfo(CallBase *, std::vector<int>);

  // Find all indirect calls
  std::vector<CallBase *> getIndirectCalls();

  std::set<Function *> getTargets(CallBase *CB, bool print = false);

  std::set<Function *> findTargetsSVF(CallBase *CB);
  std::set<Function *> findTargetsType(CallBase *CB);
  std::set<Function *> *findTargetsMLTA(CallBase *CB);

  // check if the two functions have the same type
  // renamed structs are considered the same (e.g. struct.A == struct.A.12)
  bool hasSameType(FunctionType *, FunctionType *);

  bool containsBlackHole(CallBase *CB);
};
} // namespace HPCFI

#endif // HPCFIBASE_H_