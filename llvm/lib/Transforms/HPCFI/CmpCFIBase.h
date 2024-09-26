//===-- CmpCFIBase.h  -----------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#ifndef CMPCFIBASE_H_
#define CMPCFIBASE_H_

#include "HPCFIBase.h"

using namespace llvm;

namespace HPCFI {
class CmpCFIBase : public virtual HPCFIBase {
protected:
  CmpCFIBase(char ID) : HPCFIBase(ID) {}

  // insert CMP-CFI check at indirect call
  BasicBlock *insertCmpCFICheck(CallBase *, std::set<Function *>);
};
} // namespace HPCFI

#endif // CMPCFIBASE_H_