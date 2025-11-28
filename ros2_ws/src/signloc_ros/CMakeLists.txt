cmake_minimum_required(VERSION 3.5)
project(signloc_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


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


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV ${OpenCV_MIN_VERSION} REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(signloc_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(pybind11_DIR /usr/local/lib/python3.10/dist-packages/)
set(PYTHON_INCLUDE_DIR "/usr/include/python3.10")
find_package (Python COMPONENTS Interpreter Development)
if(Python_FOUND)
  message(STATUS "Found Python version ${Python_VERSION}")
  include_directories(${PYTHON_INCLUDE_DIR})
endif(Python_FOUND)
find_package(pybind11 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#add_subdirectory("src")
add_library(CORE STATIC IMPORTED)
set_target_properties(CORE PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/signloc_core/build/lib/libCORE.a)

add_executable(MCLNode src/MCLNode.cpp src/RosUtils.cpp)
target_link_libraries(MCLNode CORE ${Boost_SYSTEM_LIBRARY} pybind11::embed)
ament_target_dependencies(MCLNode rclcpp std_msgs sensor_msgs visualization_msgs nav_msgs signloc_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs  OpenCV)



install(TARGETS
 MCLNode
DESTINATION lib/${PROJECT_NAME}
)


#install(DIRECTORY
#DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}/
)



ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
        scripts/BroadcastNavGraphNode.py
        scripts/NavSatPublisher.py
        DESTINATION lib/${PROJECT_NAME}
    )

ament_package()
