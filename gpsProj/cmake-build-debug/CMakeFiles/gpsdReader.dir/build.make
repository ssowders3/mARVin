# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /home/andrew/clion-2018.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/andrew/clion-2018.2.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/andrew/mARVin/gpsProj

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrew/mARVin/gpsProj/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/gpsdReader.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gpsdReader.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gpsdReader.dir/flags.make

CMakeFiles/gpsdReader.dir/main.cpp.o: CMakeFiles/gpsdReader.dir/flags.make
CMakeFiles/gpsdReader.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrew/mARVin/gpsProj/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gpsdReader.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gpsdReader.dir/main.cpp.o -c /home/andrew/mARVin/gpsProj/main.cpp

CMakeFiles/gpsdReader.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpsdReader.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrew/mARVin/gpsProj/main.cpp > CMakeFiles/gpsdReader.dir/main.cpp.i

CMakeFiles/gpsdReader.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpsdReader.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrew/mARVin/gpsProj/main.cpp -o CMakeFiles/gpsdReader.dir/main.cpp.s

# Object files for target gpsdReader
gpsdReader_OBJECTS = \
"CMakeFiles/gpsdReader.dir/main.cpp.o"

# External object files for target gpsdReader
gpsdReader_EXTERNAL_OBJECTS =

gpsdReader: CMakeFiles/gpsdReader.dir/main.cpp.o
gpsdReader: CMakeFiles/gpsdReader.dir/build.make
gpsdReader: CMakeFiles/gpsdReader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrew/mARVin/gpsProj/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gpsdReader"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpsdReader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gpsdReader.dir/build: gpsdReader

.PHONY : CMakeFiles/gpsdReader.dir/build

CMakeFiles/gpsdReader.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gpsdReader.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gpsdReader.dir/clean

CMakeFiles/gpsdReader.dir/depend:
	cd /home/andrew/mARVin/gpsProj/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrew/mARVin/gpsProj /home/andrew/mARVin/gpsProj /home/andrew/mARVin/gpsProj/cmake-build-debug /home/andrew/mARVin/gpsProj/cmake-build-debug /home/andrew/mARVin/gpsProj/cmake-build-debug/CMakeFiles/gpsdReader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gpsdReader.dir/depend

