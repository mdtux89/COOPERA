# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/marco/bioloid/Modules/Planner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marco/bioloid/Modules/Planner/build

# Include any dependencies generated for this target.
include CMakeFiles/Planner.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Planner.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Planner.dir/flags.make

CMakeFiles/Planner.dir/src/Planner.cpp.o: CMakeFiles/Planner.dir/flags.make
CMakeFiles/Planner.dir/src/Planner.cpp.o: ../src/Planner.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marco/bioloid/Modules/Planner/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Planner.dir/src/Planner.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Planner.dir/src/Planner.cpp.o -c /home/marco/bioloid/Modules/Planner/src/Planner.cpp

CMakeFiles/Planner.dir/src/Planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Planner.dir/src/Planner.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marco/bioloid/Modules/Planner/src/Planner.cpp > CMakeFiles/Planner.dir/src/Planner.cpp.i

CMakeFiles/Planner.dir/src/Planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Planner.dir/src/Planner.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marco/bioloid/Modules/Planner/src/Planner.cpp -o CMakeFiles/Planner.dir/src/Planner.cpp.s

CMakeFiles/Planner.dir/src/Planner.cpp.o.requires:
.PHONY : CMakeFiles/Planner.dir/src/Planner.cpp.o.requires

CMakeFiles/Planner.dir/src/Planner.cpp.o.provides: CMakeFiles/Planner.dir/src/Planner.cpp.o.requires
	$(MAKE) -f CMakeFiles/Planner.dir/build.make CMakeFiles/Planner.dir/src/Planner.cpp.o.provides.build
.PHONY : CMakeFiles/Planner.dir/src/Planner.cpp.o.provides

CMakeFiles/Planner.dir/src/Planner.cpp.o.provides.build: CMakeFiles/Planner.dir/src/Planner.cpp.o

CMakeFiles/Planner.dir/src/main.cpp.o: CMakeFiles/Planner.dir/flags.make
CMakeFiles/Planner.dir/src/main.cpp.o: ../src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marco/bioloid/Modules/Planner/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Planner.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Planner.dir/src/main.cpp.o -c /home/marco/bioloid/Modules/Planner/src/main.cpp

CMakeFiles/Planner.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Planner.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marco/bioloid/Modules/Planner/src/main.cpp > CMakeFiles/Planner.dir/src/main.cpp.i

CMakeFiles/Planner.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Planner.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marco/bioloid/Modules/Planner/src/main.cpp -o CMakeFiles/Planner.dir/src/main.cpp.s

CMakeFiles/Planner.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/Planner.dir/src/main.cpp.o.requires

CMakeFiles/Planner.dir/src/main.cpp.o.provides: CMakeFiles/Planner.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Planner.dir/build.make CMakeFiles/Planner.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/Planner.dir/src/main.cpp.o.provides

CMakeFiles/Planner.dir/src/main.cpp.o.provides.build: CMakeFiles/Planner.dir/src/main.cpp.o

# Object files for target Planner
Planner_OBJECTS = \
"CMakeFiles/Planner.dir/src/Planner.cpp.o" \
"CMakeFiles/Planner.dir/src/main.cpp.o"

# External object files for target Planner
Planner_EXTERNAL_OBJECTS =

Planner: CMakeFiles/Planner.dir/src/Planner.cpp.o
Planner: CMakeFiles/Planner.dir/src/main.cpp.o
Planner: CMakeFiles/Planner.dir/build.make
Planner: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.62.1
Planner: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.62.1
Planner: /usr/lib/x86_64-linux-gnu/libYARP_math.so.2.3.62.1
Planner: /usr/lib/x86_64-linux-gnu/libYARP_dev.so.2.3.62.1
Planner: /usr/lib/x86_64-linux-gnu/libYARP_name.so.2.3.62.1
Planner: /usr/lib/x86_64-linux-gnu/libYARP_init.so.2.3.62.1
Planner: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.62.1
Planner: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.62.1
Planner: CMakeFiles/Planner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Planner"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Planner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Planner.dir/build: Planner
.PHONY : CMakeFiles/Planner.dir/build

CMakeFiles/Planner.dir/requires: CMakeFiles/Planner.dir/src/Planner.cpp.o.requires
CMakeFiles/Planner.dir/requires: CMakeFiles/Planner.dir/src/main.cpp.o.requires
.PHONY : CMakeFiles/Planner.dir/requires

CMakeFiles/Planner.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Planner.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Planner.dir/clean

CMakeFiles/Planner.dir/depend:
	cd /home/marco/bioloid/Modules/Planner/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marco/bioloid/Modules/Planner /home/marco/bioloid/Modules/Planner /home/marco/bioloid/Modules/Planner/build /home/marco/bioloid/Modules/Planner/build /home/marco/bioloid/Modules/Planner/build/CMakeFiles/Planner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Planner.dir/depend

