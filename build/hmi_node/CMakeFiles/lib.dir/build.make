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
CMAKE_SOURCE_DIR = /home/riccardochiaretti/alba_v2_hmi/src/hmi_node

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/riccardochiaretti/alba_v2_hmi/build/hmi_node

# Include any dependencies generated for this target.
include CMakeFiles/lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lib.dir/flags.make

CMakeFiles/lib.dir/src/jsonprimitives.cpp.o: CMakeFiles/lib.dir/flags.make
CMakeFiles/lib.dir/src/jsonprimitives.cpp.o: /home/riccardochiaretti/alba_v2_hmi/src/hmi_node/src/jsonprimitives.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/riccardochiaretti/alba_v2_hmi/build/hmi_node/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lib.dir/src/jsonprimitives.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib.dir/src/jsonprimitives.cpp.o -c /home/riccardochiaretti/alba_v2_hmi/src/hmi_node/src/jsonprimitives.cpp

CMakeFiles/lib.dir/src/jsonprimitives.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib.dir/src/jsonprimitives.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/riccardochiaretti/alba_v2_hmi/src/hmi_node/src/jsonprimitives.cpp > CMakeFiles/lib.dir/src/jsonprimitives.cpp.i

CMakeFiles/lib.dir/src/jsonprimitives.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib.dir/src/jsonprimitives.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/riccardochiaretti/alba_v2_hmi/src/hmi_node/src/jsonprimitives.cpp -o CMakeFiles/lib.dir/src/jsonprimitives.cpp.s

CMakeFiles/lib.dir/src/jsonprimitives.cpp.o.requires:

.PHONY : CMakeFiles/lib.dir/src/jsonprimitives.cpp.o.requires

CMakeFiles/lib.dir/src/jsonprimitives.cpp.o.provides: CMakeFiles/lib.dir/src/jsonprimitives.cpp.o.requires
	$(MAKE) -f CMakeFiles/lib.dir/build.make CMakeFiles/lib.dir/src/jsonprimitives.cpp.o.provides.build
.PHONY : CMakeFiles/lib.dir/src/jsonprimitives.cpp.o.provides

CMakeFiles/lib.dir/src/jsonprimitives.cpp.o.provides.build: CMakeFiles/lib.dir/src/jsonprimitives.cpp.o


# Object files for target lib
lib_OBJECTS = \
"CMakeFiles/lib.dir/src/jsonprimitives.cpp.o"

# External object files for target lib
lib_EXTERNAL_OBJECTS =

liblib.a: CMakeFiles/lib.dir/src/jsonprimitives.cpp.o
liblib.a: CMakeFiles/lib.dir/build.make
liblib.a: CMakeFiles/lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/riccardochiaretti/alba_v2_hmi/build/hmi_node/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lib.dir/build: liblib.a

.PHONY : CMakeFiles/lib.dir/build

CMakeFiles/lib.dir/requires: CMakeFiles/lib.dir/src/jsonprimitives.cpp.o.requires

.PHONY : CMakeFiles/lib.dir/requires

CMakeFiles/lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lib.dir/clean

CMakeFiles/lib.dir/depend:
	cd /home/riccardochiaretti/alba_v2_hmi/build/hmi_node && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/riccardochiaretti/alba_v2_hmi/src/hmi_node /home/riccardochiaretti/alba_v2_hmi/src/hmi_node /home/riccardochiaretti/alba_v2_hmi/build/hmi_node /home/riccardochiaretti/alba_v2_hmi/build/hmi_node /home/riccardochiaretti/alba_v2_hmi/build/hmi_node/CMakeFiles/lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lib.dir/depend
