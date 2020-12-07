add_executable(wrappingSpeed
        main.cpp
        buildModelDCC.cpp)

target_link_libraries(wrappingSpeed common_options_target)
set_target_properties(wrappingSpeed PROPERTIES FOLDER "Examples")
add_test(NAME    wrappingSpeed
         COMMAND wrappingSpeed noVisualizer simulateOnce)
