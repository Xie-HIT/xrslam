if(NOT TARGET depends::glbinding)
  if(NOT TARGET options::modern-cpp)
    message(FATAL_ERROR "depends::glbinding expects options::modern-cpp")
  endif()
  FetchContent_Declare(
    depends-glbinding
    GIT_REPOSITORY https://gitee.com/xiechen-HIT/glbinding.git
    GIT_TAG        v3.1.0
  )
  FetchContent_GetProperties(depends-glbinding)
  if(NOT depends-glbinding_POPULATED)
    message(STATUS "Fetching glbinding sources")
    FetchContent_Populate(depends-glbinding)
    message(STATUS "Fetching glbinding sources - done")
  endif()
  set(OPTION_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(OPTION_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(${depends-glbinding_SOURCE_DIR} ${depends-glbinding_BINARY_DIR})
  find_package(glbinding REQUIRED PATHS "${depends-glbinding_SOURCE_DIR}" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  add_library(depends::glbinding INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::glbinding INTERFACE glbinding::glbinding glbinding::glbinding-aux options::modern-cpp)
  set(depends-glbinding-source-dir ${depends-glbinding_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  set(depends-glbinding-binary-dir ${depends-glbinding_BINARY_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-glbinding-source-dir)
  mark_as_advanced(depends-glbinding-binary-dir)
endif()
