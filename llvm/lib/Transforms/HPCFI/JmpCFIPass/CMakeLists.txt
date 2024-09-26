include_directories( ${CMAKE_SOURCE_DIR}/../../hpcfi-svf/include )

add_llvm_library( LLVMHPCFIJmp MODULE 
  JmpCFIPass.cpp
  ../HPCFIBase.cpp
  ../JmpCFIBase.cpp
  ../CmpCFIBase.cpp

  PLUGIN_TOOL
  opt
  )
