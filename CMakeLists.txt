cmake_minimum_required(VERSION 3.14)

# Set the project name
project(robotRKDLib 
    VERSION 1.0
    LANGUAGES CXX)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Install Variables
include(GNUInstallDirs)

# Compile extra options
add_compile_options(-Wall -Wextra -fPIC)

# Add the find packages for Eigen in the cmake folder
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

# Find packages
find_package(Boost REQUIRED COMPONENTS date_time filesystem iostreams)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

## Build options
option(STRICT "Treat warnings as errors" OFF)
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build example applications" OFF)
option(BUILD_SHARED_LIBS "Build shared instead of static library" OFF)

# Set variables
set(LIB_NAME rkd)

# Sources and headers
set(SOURCES src/Robot.cpp)
set(HEADERS include/rkd/Robot.h)

# Create the library
add_library(${LIB_NAME} ${SOURCES})

# Set shared/static linking
set_target_properties(${LIB_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "${HEADERS}"
)

# Ensure Eigen is propagated to downstream projects
target_link_libraries(${LIB_NAME} PUBLIC
    ${Boost_LIBRARIES}
    urdfdom_model
    orocos-kdl
    Eigen3::Eigen 
    kdl_parser
    trac_ik
    pthread
    ${pkg_nlopt_LIBRARIES}
)

# Also tell CMake to export Eigen include paths for downstream projects
target_include_directories(${LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIR}
)
# ---------------------- GoogleTest (Do NOT install) ----------------------
# if (BUILD_TESTS)
#   add_subdirectory(dep/gtest)
#   enable_testing()

#   add_executable(test_robot tests/test_robot.cpp)
#   target_link_libraries(test_robot PRIVATE GTest::gtest_main rkd)

#   include(GoogleTest)
#   gtest_discover_tests(test_robot)
# endif()

# ---------------------- Example App (Verifies that linking works) ----------------------
if (BUILD_EXAMPLES)
  add_executable(example_script examples/example_init_class.cpp)
  target_link_libraries(example_script PRIVATE rkd)
endif()

# ---------------------- Installation ----------------------

install(TARGETS ${LIB_NAME}
    EXPORT ${LIB_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rkd
)

# Generate and install CMake package files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${LIB_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}Config.cmake
    @ONLY
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${LIB_NAME}/cmake
)

install(EXPORT ${LIB_NAME}Targets
    FILE ${LIB_NAME}Targets.cmake
    NAMESPACE ${LIB_NAME}::
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${LIB_NAME}/cmake
)
