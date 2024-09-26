set(LLVM_LINK_COMPONENTS Demangle)

include_directories( ${CMAKE_SOURCE_DIR}/../../hpcfi-svf/include )

add_llvm_library( LLVMMLTA MODULE 
  MLTAPass.cpp
  MLTA.cpp

  PLUGIN_TOOL
  opt
  )
