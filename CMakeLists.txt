cmake_minimum_required(VERSION 3.2)
project(OpenSim_RA)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# just put the binaries into the build output dir, rather than into a
# subfolder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# so that CLion/qtcreator can open your project without further
# configuring
list(APPEND CMAKE_PREFIX_PATH "/home/none/opensim-core/lib/cmake/")

find_package(OpenSim REQUIRED)
find_package(Simbody REQUIRED)

# any target that links to this INTERFACE target will transitively
# inherit all the library, include, compile, etc. options of that
# target. It's a handy trick for configuring all the dependencies +
# options on one target, rather than having to duplicate it
add_library(common_options_target INTERFACE)
target_link_libraries(common_options_target INTERFACE
  SimTKcommon
  SimTKmath
  SimTKsimbody
  osimTools)

# this uses generator expressions. see (find "COMPILER_ID")
#     https://cmake.org/cmake/help/latest/manual/cmake-generator-expressions.7.html
target_compile_options(common_options_target INTERFACE
  # The answers file uses `#pragma region` to facilitate code folding
  # when using this example in a live demonstration, but these pragmas
  # are only recognized by MSVC.
  "$<$<CXX_COMPILER_ID:GNU>:-Wno-unknown-pragmas>"
  "$<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas>")

add_subdirectory(Analytical_wrapping)
add_subdirectory(Wrapping_surfaces)
add_subdirectory(Cables)
add_subdirectory(Wrapping_speed)
