


add_executable(UnitTests TestGraph.cpp JSONLoader.cpp)
target_link_libraries(UnitTests ${OpenCV_LIBS} ${catkin_LIBRARIES} CORE GTest::GTest gtest_main ${Boost_LIBRARIES} pybind11::embed)
target_compile_definitions(UnitTests PRIVATE PROJECT_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
add_test(AllTestsInTests UnitTests)


add_executable(Eval JSONLoader.cpp EvalMCL.cpp)
target_link_libraries(Eval ${OpenCV_LIBS} ${catkin_LIBRARIES} CORE ${Boost_LIBRARIES} pybind11::embed)




