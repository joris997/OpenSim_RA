add_executable(ellipsoidTest
    main.cpp
    buildModel.cpp
    buildModelOld.cpp
    analyticalSolution.cpp)
target_link_libraries(ellipsoidTest common_options_target)
set_target_properties(ellipsoidTest PROPERTIES FOLDER "Examples")
add_test(NAME    ellipsoidTest
         COMMAND ellipsoidTest noVisualizer simulateOnce)
