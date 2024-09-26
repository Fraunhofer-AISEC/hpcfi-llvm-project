include_directories( ${CMAKE_SOURCE_DIR}/../../hpcfi-svf/include )

add_llvm_library( LLVMHPCFICmp MODULE 
  CmpCFIPass.cpp
  ../HPCFIBase.cpp
  ../CmpCFIBase.cpp
  
  PLUGIN_TOOL
  opt
  )
