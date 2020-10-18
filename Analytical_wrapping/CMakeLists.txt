add_executable(test_suite
        test_suite.cpp
        buildModel.cpp
        buildModelPathPoints.cpp
        buildModelHorizontal.cpp
        buildModelDouble.cpp
        analyticalSolution.cpp)

target_link_libraries(test_suite common_options_target)
set_target_properties(test_suite PROPERTIES FOLDER "Examples")
add_test(NAME    test_suite
         COMMAND test_suite noVisualizer simulateOnce)
