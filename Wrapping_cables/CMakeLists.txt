#cmake_minimum_required(VERSION 3.2)
#project(Cable_test)
#
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(Simbody_DIR "/home/none/simbody/lib/cmake/simbody")
#find_package(Simbody REQUIRED)
#
#add_executable(Cable_test
#        OpenSimPartyDemoCable.cpp)
#target_link_libraries(Cable_test ${Simbody_LIBRARIES})
#set_target_properties(Cable_test PROPERTIES FOLDER "Examples")
#add_test(NAME    Cable_test
#        COMMAND Cable_test noVisualizer simulateOnce)

add_executable(Cables_test
               Cables_Simbody_demo.cpp)
#               Cables_test.cpp)
target_link_libraries(Cables_test common_options_target)
set_target_properties(Cables_test PROPERTIES FOLDER "Examples")
add_test(NAME    Cables_test
         COMMAND Cables_test noVisualizer simulateOnce)
