cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

# set the project name
project(robotRKDLib 
    VERSION 1.0
    LANGUAGES CXX)

# # specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Install Variables
include(GNUInstallDirs)

# compile extras options
add_compile_options(-Wall -Wextra -O3)

# add the find packages for Eigen in the cmake folder
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

# find packages
find_package(Boost REQUIRED COMPONENTS date_time filesystem iostreams)
find_package(Eigen3 REQUIRED)

## Build options
option(STRICT "Treat warnings as errors" OFF)
option(COMPILE_EXAMPLE "Compile the test function" ON)

# file to compile
set(EXECUTABLE_FILES
  src/Robot.cpp
)

# Set variables
set(LIB_NAME RKD_R)

# add Libraries
add_library(${LIB_NAME} SHARED src/Robot.cpp)

# Include Dirs
target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} )

# Target link libraries
target_link_libraries(${LIB_NAME} PRIVATE ${Boost_LIBRARIES} urdfdom_model orocos-kdl Eigen3::Eigen3 kdl_parser trac_ik pthread )

# Headers
set(HEADERS_RKD
  include/RKD/Robot.h
)

# ------------------------------------------------------- TEST CODE
if (COMPILE_EXAMPLE)
  # add the executable
  add_executable(test_script MACOSX_BUNDLE src/test.cpp)

  # Include Dirs
  target_include_directories(test_script PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} )

  # Target link libraries
  target_link_libraries(test_script PRIVATE Boost::date_time Boost::filesystem Boost::iostreams urdfdom_model orocos-kdl Eigen3::Eigen3 kdl_parser trac_ik ${LIB_NAME} pthread)
endif()
# ------------------------------------------------------- end TEST CODE


# ------------------------------------------------------- INSTALL

# Set Version properties
set_target_properties(${LIB_NAME} PROPERTIES VERSION ${PROJECT_VERSION} 
								PUBLIC_HEADER "${HEADERS_RKD}")

# Install target
install(TARGETS ${LIB_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RKD/)