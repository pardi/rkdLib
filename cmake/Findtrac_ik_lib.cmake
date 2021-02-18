find_package(trac_ik_lib CONFIG)
mark_as_advanced(FORCE trac_ik_lib_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(trac_ik_lib
  FOUND_VAR trac_ik_lib_FOUND
  REQUIRED_VARS trac_ik_lib_INCLUDE_DIRS
)

if(NOT TARGET trac_ik_lib::trac_ik_lib)
  add_library(trac_ik_lib::trac_ik_lib INTERFACE IMPORTED)
  set_target_properties(trac_ik_lib::trac_ik_lib PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS}
    INTERFACE_COMPILE_DEFINITIONS "${EIGEN3_DEFINITIONS}"
  )
endif()
