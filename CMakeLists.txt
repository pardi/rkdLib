cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

# set the project name
project(RKDLib 
    VERSION 1.0
    LANGUAGES CXX)

# # specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# compile extras options
add_compile_options(-Wall -Wextra -O3)

# add the find packages for Eigen in the cmake folder
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

# find packages
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trac_ik_lib REQUIRED)

## Build options
option(STRICT "Treat warnings as errors" OFF)

# file to compile
set(EXECUTABLE_FILES
  src/robot.cpp
  #src/test.cpp
)

# Set variables
set(LIB_NAME rkdRobot)

# add Libraries
add_library(${LIB_NAME} SHARED src/robot.cpp)

# Headers
set(HEADERS_RKD
	include/RKD/robot.h
)

# add the executable
#add_executable(${LIB_NAME}  "${EXECUTABLE_FILES}")

# Include Dirs
target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

# Target link libraries
target_link_libraries(${LIB_NAME} PRIVATE ${Boost_LIBRARIES} urdfdom_model orocos-kdl Eigen3::Eigen3 kdl_parser trac_ik)

# ------------------------------------------------------- INSTALL

# Set Version properties
set_target_properties(${LIB_NAME} PROPERTIES VERSION ${PROJECT_VERSION} 
								PUBLIC_HEADER "${HEADERS_RKD}")

# Install target
install(TARGETS ${LIB_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION RKD/${CMAKE_INSTALL_INCLUDEDIR})