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
CMAKE_SOURCE_DIR = /home/marco/bioloid/Modules/Sensors

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marco/bioloid/Modules/Sensors/build

# Include any dependencies generated for this target.
include CMakeFiles/Sensors.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Sensors.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Sensors.dir/flags.make

CMakeFiles/Sensors.dir/src/main.cpp.o: CMakeFiles/Sensors.dir/flags.make
CMakeFiles/Sensors.dir/src/main.cpp.o: ../src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marco/bioloid/Modules/Sensors/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Sensors.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Sensors.dir/src/main.cpp.o -c /home/marco/bioloid/Modules/Sensors/src/main.cpp

CMakeFiles/Sensors.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sensors.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marco/bioloid/Modules/Sensors/src/main.cpp > CMakeFiles/Sensors.dir/src/main.cpp.i

CMakeFiles/Sensors.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sensors.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marco/bioloid/Modules/Sensors/src/main.cpp -o CMakeFiles/Sensors.dir/src/main.cpp.s

CMakeFiles/Sensors.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/Sensors.dir/src/main.cpp.o.requires

CMakeFiles/Sensors.dir/src/main.cpp.o.provides: CMakeFiles/Sensors.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Sensors.dir/build.make CMakeFiles/Sensors.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/Sensors.dir/src/main.cpp.o.provides

CMakeFiles/Sensors.dir/src/main.cpp.o.provides.build: CMakeFiles/Sensors.dir/src/main.cpp.o

CMakeFiles/Sensors.dir/src/Sensors.cpp.o: CMakeFiles/Sensors.dir/flags.make
CMakeFiles/Sensors.dir/src/Sensors.cpp.o: ../src/Sensors.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marco/bioloid/Modules/Sensors/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Sensors.dir/src/Sensors.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Sensors.dir/src/Sensors.cpp.o -c /home/marco/bioloid/Modules/Sensors/src/Sensors.cpp

CMakeFiles/Sensors.dir/src/Sensors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sensors.dir/src/Sensors.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marco/bioloid/Modules/Sensors/src/Sensors.cpp > CMakeFiles/Sensors.dir/src/Sensors.cpp.i

CMakeFiles/Sensors.dir/src/Sensors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sensors.dir/src/Sensors.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marco/bioloid/Modules/Sensors/src/Sensors.cpp -o CMakeFiles/Sensors.dir/src/Sensors.cpp.s

CMakeFiles/Sensors.dir/src/Sensors.cpp.o.requires:
.PHONY : CMakeFiles/Sensors.dir/src/Sensors.cpp.o.requires

CMakeFiles/Sensors.dir/src/Sensors.cpp.o.provides: CMakeFiles/Sensors.dir/src/Sensors.cpp.o.requires
	$(MAKE) -f CMakeFiles/Sensors.dir/build.make CMakeFiles/Sensors.dir/src/Sensors.cpp.o.provides.build
.PHONY : CMakeFiles/Sensors.dir/src/Sensors.cpp.o.provides

CMakeFiles/Sensors.dir/src/Sensors.cpp.o.provides.build: CMakeFiles/Sensors.dir/src/Sensors.cpp.o

# Object files for target Sensors
Sensors_OBJECTS = \
"CMakeFiles/Sensors.dir/src/main.cpp.o" \
"CMakeFiles/Sensors.dir/src/Sensors.cpp.o"

# External object files for target Sensors
Sensors_EXTERNAL_OBJECTS =

Sensors: CMakeFiles/Sensors.dir/src/main.cpp.o
Sensors: CMakeFiles/Sensors.dir/src/Sensors.cpp.o
Sensors: CMakeFiles/Sensors.dir/build.make
Sensors: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.62.1
Sensors: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.62.1
Sensors: /usr/lib/x86_64-linux-gnu/libYARP_math.so.2.3.62.1
Sensors: /usr/lib/x86_64-linux-gnu/libYARP_dev.so.2.3.62.1
Sensors: /usr/lib/x86_64-linux-gnu/libYARP_name.so.2.3.62.1
Sensors: /usr/lib/x86_64-linux-gnu/libYARP_init.so.2.3.62.1
Sensors: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.62.1
Sensors: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.62.1
Sensors: CMakeFiles/Sensors.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Sensors"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sensors.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Sensors.dir/build: Sensors
.PHONY : CMakeFiles/Sensors.dir/build

CMakeFiles/Sensors.dir/requires: CMakeFiles/Sensors.dir/src/main.cpp.o.requires
CMakeFiles/Sensors.dir/requires: CMakeFiles/Sensors.dir/src/Sensors.cpp.o.requires
.PHONY : CMakeFiles/Sensors.dir/requires

CMakeFiles/Sensors.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Sensors.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Sensors.dir/clean

CMakeFiles/Sensors.dir/depend:
	cd /home/marco/bioloid/Modules/Sensors/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marco/bioloid/Modules/Sensors /home/marco/bioloid/Modules/Sensors /home/marco/bioloid/Modules/Sensors/build /home/marco/bioloid/Modules/Sensors/build /home/marco/bioloid/Modules/Sensors/build/CMakeFiles/Sensors.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Sensors.dir/depend

