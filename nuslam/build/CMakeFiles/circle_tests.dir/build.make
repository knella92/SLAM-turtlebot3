# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/build

# Include any dependencies generated for this target.
include CMakeFiles/circle_tests.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/circle_tests.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/circle_tests.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/circle_tests.dir/flags.make

CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o: CMakeFiles/circle_tests.dir/flags.make
CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o: ../tests/circle_tests.cpp
CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o: CMakeFiles/circle_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o -MF CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o.d -o CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o -c /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/tests/circle_tests.cpp

CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/tests/circle_tests.cpp > CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.i

CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/tests/circle_tests.cpp -o CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.s

# Object files for target circle_tests
circle_tests_OBJECTS = \
"CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o"

# External object files for target circle_tests
circle_tests_EXTERNAL_OBJECTS =

circle_tests: CMakeFiles/circle_tests.dir/tests/circle_tests.cpp.o
circle_tests: CMakeFiles/circle_tests.dir/build.make
circle_tests: /usr/local/lib/libCatch2Main.a
circle_tests: /home/kevin/workspaces/SLAM/hw1/install/turtlelib/lib/libturtlelib.a
circle_tests: /usr/local/lib/libCatch2.a
circle_tests: /usr/lib/libarmadillo.so
circle_tests: CMakeFiles/circle_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable circle_tests"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/circle_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/circle_tests.dir/build: circle_tests
.PHONY : CMakeFiles/circle_tests.dir/build

CMakeFiles/circle_tests.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/circle_tests.dir/cmake_clean.cmake
.PHONY : CMakeFiles/circle_tests.dir/clean

CMakeFiles/circle_tests.dir/depend:
	cd /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/build /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/build /home/kevin/workspaces/SLAM/hw1/src/nuturtle-knella92/nuslam/build/CMakeFiles/circle_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/circle_tests.dir/depend

