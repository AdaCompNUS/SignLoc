




add_executable(MCLNode MCLNode.cpp RosUtils.cpp)
target_link_libraries(MCLNode ${OpenCV_LIBS} ${catkin_LIBRARIES} CORE ${Boost_LIBRARIES} pybind11::embed)


