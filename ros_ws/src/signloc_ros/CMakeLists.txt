cmake_minimum_required(VERSION 3.0.2)
project(signloc_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/lib/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/signloc_core/include/)

set(RESULTS_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/results)
file(MAKE_DIRECTORY ${RESULTS_OUTPUT_DIRECTORY})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${RESULTS_OUTPUT_DIRECTORY}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${RESULTS_OUTPUT_DIRECTORY}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RESULTS_OUTPUT_DIRECTORY}/bin)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/signloc_core/build/lib)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(
  catkin REQUIRED COMPONENTS roscpp sensor_msgs message_filters geometry_msgs nav_msgs tf2 tf cv_bridge pcl_ros signloc_msgs)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
  message(STATUS "OpenCV directories: ${OpenCV_INCLUDE_DIRS}")
  include_directories(${OpenCV_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "OpenCV not found, please read the README.md")
endif(OpenCV_FOUND)

find_package (Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)
if(Python_FOUND)
  message(STATUS "Found Python version ${Python_VERSION}")
  include_directories(${PYTHON_INCLUDE_DIR})
endif(Python_FOUND)
find_package(pybind11 REQUIRED)

find_package(Boost COMPONENTS serialization system filesystem)
if(Boost_FOUND)
  message(STATUS "Found Boost version ${Boost_VERSION}")
  message(STATUS "Boost directories: ${Boost_INCLUDE_DIRS}")
  include_directories(${Boost_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Boost not found, please read the README.md")
endif(Boost_FOUND)





###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES graphloc_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


add_subdirectory("src")
add_library(CORE STATIC IMPORTED)
set_target_properties(CORE PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/signloc_core/build/lib/libCORE.a)

