cmake_minimum_required(VERSION 3.2)
project(Analytical_wrapping)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# DEFINE HERE THE LOCATION OF YOUR OPENSIM-CORE INSTALLATION!!!
set(OpenSim_DIR "/home/none/opensim-core/lib/cmake/OpenSim")
find_package(OpenSim REQUIRED)


add_executable(test_suite
    test_suite.cpp
    buildModel.cpp)
target_link_libraries(test_suite ${Simbody_LIBRARIES} osimTools)
set_target_properties(test_suite PROPERTIES FOLDER "Examples")
add_test(NAME    test_suite
         COMMAND test_suite noVisualizer simulateOnce)

#target_link_libraries(exampleHopperDeviceAnswers ${Simbody_LIBRARIES} osimTools)
#set_target_properties(exampleHopperDeviceAnswers PROPERTIES FOLDER "Examples")
# The answers file uses `#pragma region` to facilitate code folding
# when using this example in a live demonstration, but these pragmas are only
# recognized by MSVC.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(test_suite PUBLIC
            -Wno-unknown-pragmas)
endif()
#add_test(NAME    exampleHopperDeviceAnswers
#         COMMAND exampleHopperDeviceAnswers noVisualizer)
