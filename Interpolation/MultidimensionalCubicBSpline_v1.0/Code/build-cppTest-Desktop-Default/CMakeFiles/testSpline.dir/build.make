# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default

# Include any dependencies generated for this target.
include CMakeFiles/testSpline.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testSpline.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testSpline.dir/flags.make

CMakeFiles/testSpline.dir/testSpline.cpp.o: CMakeFiles/testSpline.dir/flags.make
CMakeFiles/testSpline.dir/testSpline.cpp.o: /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest/testSpline.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testSpline.dir/testSpline.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testSpline.dir/testSpline.cpp.o -c /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest/testSpline.cpp

CMakeFiles/testSpline.dir/testSpline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testSpline.dir/testSpline.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest/testSpline.cpp > CMakeFiles/testSpline.dir/testSpline.cpp.i

CMakeFiles/testSpline.dir/testSpline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testSpline.dir/testSpline.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest/testSpline.cpp -o CMakeFiles/testSpline.dir/testSpline.cpp.s

CMakeFiles/testSpline.dir/testSpline.cpp.o.requires:

.PHONY : CMakeFiles/testSpline.dir/testSpline.cpp.o.requires

CMakeFiles/testSpline.dir/testSpline.cpp.o.provides: CMakeFiles/testSpline.dir/testSpline.cpp.o.requires
	$(MAKE) -f CMakeFiles/testSpline.dir/build.make CMakeFiles/testSpline.dir/testSpline.cpp.o.provides.build
.PHONY : CMakeFiles/testSpline.dir/testSpline.cpp.o.provides

CMakeFiles/testSpline.dir/testSpline.cpp.o.provides.build: CMakeFiles/testSpline.dir/testSpline.cpp.o


CMakeFiles/testSpline.dir/SplineDataNew.cpp.o: CMakeFiles/testSpline.dir/flags.make
CMakeFiles/testSpline.dir/SplineDataNew.cpp.o: /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest/SplineDataNew.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/testSpline.dir/SplineDataNew.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testSpline.dir/SplineDataNew.cpp.o -c /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest/SplineDataNew.cpp

CMakeFiles/testSpline.dir/SplineDataNew.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testSpline.dir/SplineDataNew.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest/SplineDataNew.cpp > CMakeFiles/testSpline.dir/SplineDataNew.cpp.i

CMakeFiles/testSpline.dir/SplineDataNew.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testSpline.dir/SplineDataNew.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest/SplineDataNew.cpp -o CMakeFiles/testSpline.dir/SplineDataNew.cpp.s

CMakeFiles/testSpline.dir/SplineDataNew.cpp.o.requires:

.PHONY : CMakeFiles/testSpline.dir/SplineDataNew.cpp.o.requires

CMakeFiles/testSpline.dir/SplineDataNew.cpp.o.provides: CMakeFiles/testSpline.dir/SplineDataNew.cpp.o.requires
	$(MAKE) -f CMakeFiles/testSpline.dir/build.make CMakeFiles/testSpline.dir/SplineDataNew.cpp.o.provides.build
.PHONY : CMakeFiles/testSpline.dir/SplineDataNew.cpp.o.provides

CMakeFiles/testSpline.dir/SplineDataNew.cpp.o.provides.build: CMakeFiles/testSpline.dir/SplineDataNew.cpp.o


CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o: CMakeFiles/testSpline.dir/flags.make
CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o: /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o -c /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp

CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp > CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.i

CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp -o CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.s

CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o.requires:

.PHONY : CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o.requires

CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o.provides: CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o.requires
	$(MAKE) -f CMakeFiles/testSpline.dir/build.make CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o.provides.build
.PHONY : CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o.provides

CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o.provides.build: CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o


# Object files for target testSpline
testSpline_OBJECTS = \
"CMakeFiles/testSpline.dir/testSpline.cpp.o" \
"CMakeFiles/testSpline.dir/SplineDataNew.cpp.o" \
"CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o"

# External object files for target testSpline
testSpline_EXTERNAL_OBJECTS =

testSpline: CMakeFiles/testSpline.dir/testSpline.cpp.o
testSpline: CMakeFiles/testSpline.dir/SplineDataNew.cpp.o
testSpline: CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o
testSpline: CMakeFiles/testSpline.dir/build.make
testSpline: CMakeFiles/testSpline.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable testSpline"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testSpline.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testSpline.dir/build: testSpline

.PHONY : CMakeFiles/testSpline.dir/build

CMakeFiles/testSpline.dir/requires: CMakeFiles/testSpline.dir/testSpline.cpp.o.requires
CMakeFiles/testSpline.dir/requires: CMakeFiles/testSpline.dir/SplineDataNew.cpp.o.requires
CMakeFiles/testSpline.dir/requires: CMakeFiles/testSpline.dir/home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/src/SplineBasisFunction.cpp.o.requires

.PHONY : CMakeFiles/testSpline.dir/requires

CMakeFiles/testSpline.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testSpline.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testSpline.dir/clean

CMakeFiles/testSpline.dir/depend:
	cd /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/cppTest /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default /home/none/Documents/cpp/OpenSim/OpenSim_RA/Interpolation/MultidimensionalCubicBSpline_v1.0/Code/build-cppTest-Desktop-Default/CMakeFiles/testSpline.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testSpline.dir/depend
