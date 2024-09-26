//===-- MLTA.h  -----------------------------------------------------------===//

//
 

// This file is distributed under the Apache License v2.0
// License with LLVM Exceptions. See LICENSE.TXT for details.
//
// Author: Florian Kasten, Fraunhofer AISEC
//
//===----------------------------------------------------------------------===//
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Module.h"

#include <set>

using namespace llvm;

namespace HPCFI {

#define MLTA_INVALID "MLTA_INVALID"
#define MLTA_STRUCT_INFOS "struct_infos_"
#define MLTA_GEPS_IN_FPTRARRARGS "geps_in_fptrarrargs_"
#define MLTA_FPTRARR_ARG_INITS "fptrarr_arg_inits_"
#define MLTA_MEMCPY_ALIASES "memcpy_aliases_"
#define MLTA_INFO "mlta_info"

struct MLTA {
public:
  MLTA(Module &M);

  // struct_name, offsets -> functions
  std::map<std::pair<std::string, std::vector<int>>, std::set<std::string>> field_infos;

  // struct_name, offset -> function, argno
  std::map<std::pair<std::string, int>, std::set<std::pair<std::string, unsigned>>>
      geps_in_fptrarrargs;

  // (function, argno) -> offsets -> functions
  std::map<std::pair<std::string, unsigned>, std::map<std::vector<int>, std::set<std::string>>>
      fptrarr_arg_inits;

  // (dst_struct_name, dst_offset) and (src_struct_name, src_offset)
  std::set<std::pair<std::pair<std::string, int>, std::pair<std::string, int>>> memcpy_aliases;

  // try to get the struct type and offset from which a function pointer used in the indirect call
  // CB was loaded
  std::pair<std::string, std::vector<int>> getStructTypeAndOffset(const CallBase *CB);

private:
  std::map<CallBase *, std::set<std::pair<std::string, int>>> memcpy_sources;
  std::map<CallBase *, std::set<std::pair<std::string, int>>> memcpy_dests;

  // find all stores of function pointers into structs and update `field_infos`
  void setStores(Module &M);

  // find all function pointers stored in globals
  void traverseGlobals(Module &M);

  // update MLTA infos with GEP
  std::map<std::vector<int>, std::set<std::string>>
  traverseGeps(Value *GEP, std::set<std::pair<std::string, unsigned>> *FptrArrayArgs,
               std::set<CallBase *> *memcpy_dests, std::set<CallBase *> *memcpy_sources);

  // find and set all function pointer array args
  void setFunctionPointerArrayArgs(Module &M);

  std::set<std::string> followGEPToStore(Value *GEP,
                                         std::set<std::pair<std::string, unsigned>> *FptrArrayArgs,
                                         std::set<CallBase *> *memcpy_dests,
                                         std::set<CallBase *> *memcpy_sources);
  void traverseGlobal(GlobalVariable *GV, Constant *C, ConstantStruct *last_struct,
                      int last_offset);
};
} // namespace HPCFI