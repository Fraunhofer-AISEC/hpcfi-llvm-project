//===-- JmpCFIBase.h  -----------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#ifndef JMPCFIBASE_H_
#define JMPCFIBASE_H_

#include "HPCFIBase.h"

using namespace llvm;

namespace HPCFI {
typedef std::set<Function *> FuncSet;

class JmpCFIBase : public virtual HPCFIBase {
protected:
  JmpCFIBase(char ID) : HPCFIBase(ID) {}

  // Creates a jumptable containing all functions in the taret set.
  // Replaces all function pointers with the corresponding jumptable entry.
  Function *createJumptableAndReplaceFptrs(FuncSet &, unsigned);

  void insertJtCFICheck(CallBase *, Function *, BasicBlock *, BasicBlock *, BasicBlock *, unsigned);
};
} // namespace HPCFI

#endif // JMPCFIBASE_H_