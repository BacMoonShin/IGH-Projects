# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/imc/Documents/EtherCAT_CXX

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/imc/Documents/EtherCAT_CXX/build

# Include any dependencies generated for this target.
include CMakeFiles/EtherCAT_Demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/EtherCAT_Demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/EtherCAT_Demo.dir/flags.make

CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.o: CMakeFiles/EtherCAT_Demo.dir/flags.make
CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.o: ../EL2889.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/imc/Documents/EtherCAT_CXX/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.o -c /home/imc/Documents/EtherCAT_CXX/EL2889.cpp

CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/imc/Documents/EtherCAT_CXX/EL2889.cpp > CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.i

CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/imc/Documents/EtherCAT_CXX/EL2889.cpp -o CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.s

CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.o: CMakeFiles/EtherCAT_Demo.dir/flags.make
CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.o: ../EtherCAT_Demo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/imc/Documents/EtherCAT_CXX/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.o -c /home/imc/Documents/EtherCAT_CXX/EtherCAT_Demo.cpp

CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/imc/Documents/EtherCAT_CXX/EtherCAT_Demo.cpp > CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.i

CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/imc/Documents/EtherCAT_CXX/EtherCAT_Demo.cpp -o CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.s

CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.o: CMakeFiles/EtherCAT_Demo.dir/flags.make
CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.o: ../EtherCAT_Master.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/imc/Documents/EtherCAT_CXX/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.o -c /home/imc/Documents/EtherCAT_CXX/EtherCAT_Master.cpp

CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/imc/Documents/EtherCAT_CXX/EtherCAT_Master.cpp > CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.i

CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/imc/Documents/EtherCAT_CXX/EtherCAT_Master.cpp -o CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.s

# Object files for target EtherCAT_Demo
EtherCAT_Demo_OBJECTS = \
"CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.o" \
"CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.o" \
"CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.o"

# External object files for target EtherCAT_Demo
EtherCAT_Demo_EXTERNAL_OBJECTS =

../EtherCAT_Demo: CMakeFiles/EtherCAT_Demo.dir/EL2889.cpp.o
../EtherCAT_Demo: CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Demo.cpp.o
../EtherCAT_Demo: CMakeFiles/EtherCAT_Demo.dir/EtherCAT_Master.cpp.o
../EtherCAT_Demo: CMakeFiles/EtherCAT_Demo.dir/build.make
../EtherCAT_Demo: CMakeFiles/EtherCAT_Demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/imc/Documents/EtherCAT_CXX/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../EtherCAT_Demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/EtherCAT_Demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/EtherCAT_Demo.dir/build: ../EtherCAT_Demo

.PHONY : CMakeFiles/EtherCAT_Demo.dir/build

CMakeFiles/EtherCAT_Demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/EtherCAT_Demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/EtherCAT_Demo.dir/clean

CMakeFiles/EtherCAT_Demo.dir/depend:
	cd /home/imc/Documents/EtherCAT_CXX/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/imc/Documents/EtherCAT_CXX /home/imc/Documents/EtherCAT_CXX /home/imc/Documents/EtherCAT_CXX/build /home/imc/Documents/EtherCAT_CXX/build /home/imc/Documents/EtherCAT_CXX/build/CMakeFiles/EtherCAT_Demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/EtherCAT_Demo.dir/depend

