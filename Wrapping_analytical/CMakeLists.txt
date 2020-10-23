add_executable(testSuite
        main.cpp
        buildModel.cpp
        #buildModelPathPoints.cpp
        #buildModelHorizontal.cpp
        buildModelDouble.cpp
        analyticalSolution.cpp)

target_link_libraries(testSuite common_options_target)
set_target_properties(testSuite PROPERTIES FOLDER "Examples")
add_test(NAME    testSuite
         COMMAND testSuite noVisualizer simulateOnce)
