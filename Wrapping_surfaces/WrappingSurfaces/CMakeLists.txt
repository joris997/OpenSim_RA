add_executable(wrappingSurfaces
    main.cpp
    buildModel.cpp)
target_link_libraries(wrappingSurfaces common_options_target)
set_target_properties(wrappingSurfaces PROPERTIES FOLDER "Examples")
add_test(NAME    wrappingSurfaces
         COMMAND wrappingSurfaces noVisualizer simulateOnce)
