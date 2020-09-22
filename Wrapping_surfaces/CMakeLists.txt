add_executable(exampleHopperDevice
    exampleHopperDevice.cpp
    buildHopperModel.cpp)
target_link_libraries(exampleHopperDevice common_options_target)
set_target_properties(exampleHopperDevice PROPERTIES FOLDER "Examples")
add_test(NAME    exampleHopperDevice
         COMMAND exampleHopperDevice noVisualizer simulateOnce)
