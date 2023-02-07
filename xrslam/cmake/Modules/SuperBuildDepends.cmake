include(CMakeParseArguments)
include(FetchContent)

if(NOT TARGET options::modern-cpp)
  add_library(options::cpp17 INTERFACE IMPORTED GLOBAL)
  target_compile_features(options::cpp17 INTERFACE cxx_std_17)
  add_library(options::modern-cpp INTERFACE IMPORTED GLOBAL)
  target_link_libraries(options::modern-cpp INTERFACE options::cpp17)
endif()

if(NOT COMMAND superbuild_depend)
  function(superbuild_depend depend_name)
    include("${CMAKE_SOURCE_DIR}/cmake/external/${depend_name}.cmake")
  endfunction()
endif()
