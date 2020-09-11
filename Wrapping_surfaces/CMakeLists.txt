cmake_minimum_required(VERSION 3.2)
project(Wrapping_surfaces)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# DEFINE HERE THE LOCATION OF YOUR OPENSIM-CORE INSTALLATION!!!
set(OpenSim_DIR "/home/none/opensim-core/lib/cmake/OpenSim")
find_package(OpenSim REQUIRED)


add_executable(exampleHopperDevice
    exampleHopperDevice.cpp
    buildHopperModel.cpp)
target_link_libraries(exampleHopperDevice ${Simbody_LIBRARIES} osimTools)
set_target_properties(exampleHopperDevice PROPERTIES FOLDER "Examples")
add_test(NAME    exampleHopperDevice
         COMMAND exampleHopperDevice noVisualizer simulateOnce)

#target_link_libraries(exampleHopperDeviceAnswers ${Simbody_LIBRARIES} osimTools)
#set_target_properties(exampleHopperDeviceAnswers PROPERTIES FOLDER "Examples")
# The answers file uses `#pragma region` to facilitate code folding
# when using this example in a live demonstration, but these pragmas are only
# recognized by MSVC.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(exampleHopperDevice PUBLIC
            -Wno-unknown-pragmas)
endif()
#add_test(NAME    exampleHopperDeviceAnswers
#         COMMAND exampleHopperDeviceAnswers noVisualizer)
