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
CMAKE_SOURCE_DIR = /home/kabakov/dev/git/demorobot/stm32/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kabakov/dev/git/demorobot/stm32/test/build

# Include any dependencies generated for this target.
include CMakeFiles/logic.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/logic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/logic.dir/flags.make

CMakeFiles/logic.dir/logic.c.o: CMakeFiles/logic.dir/flags.make
CMakeFiles/logic.dir/logic.c.o: ../logic.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kabakov/dev/git/demorobot/stm32/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/logic.dir/logic.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/logic.dir/logic.c.o   -c /home/kabakov/dev/git/demorobot/stm32/test/logic.c

CMakeFiles/logic.dir/logic.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/logic.dir/logic.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kabakov/dev/git/demorobot/stm32/test/logic.c > CMakeFiles/logic.dir/logic.c.i

CMakeFiles/logic.dir/logic.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/logic.dir/logic.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kabakov/dev/git/demorobot/stm32/test/logic.c -o CMakeFiles/logic.dir/logic.c.s

CMakeFiles/logic.dir/logic.c.o.requires:

.PHONY : CMakeFiles/logic.dir/logic.c.o.requires

CMakeFiles/logic.dir/logic.c.o.provides: CMakeFiles/logic.dir/logic.c.o.requires
	$(MAKE) -f CMakeFiles/logic.dir/build.make CMakeFiles/logic.dir/logic.c.o.provides.build
.PHONY : CMakeFiles/logic.dir/logic.c.o.provides

CMakeFiles/logic.dir/logic.c.o.provides.build: CMakeFiles/logic.dir/logic.c.o


# Object files for target logic
logic_OBJECTS = \
"CMakeFiles/logic.dir/logic.c.o"

# External object files for target logic
logic_EXTERNAL_OBJECTS =

liblogic.a: CMakeFiles/logic.dir/logic.c.o
liblogic.a: CMakeFiles/logic.dir/build.make
liblogic.a: CMakeFiles/logic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kabakov/dev/git/demorobot/stm32/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library liblogic.a"
	$(CMAKE_COMMAND) -P CMakeFiles/logic.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/logic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/logic.dir/build: liblogic.a

.PHONY : CMakeFiles/logic.dir/build

CMakeFiles/logic.dir/requires: CMakeFiles/logic.dir/logic.c.o.requires

.PHONY : CMakeFiles/logic.dir/requires

CMakeFiles/logic.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/logic.dir/cmake_clean.cmake
.PHONY : CMakeFiles/logic.dir/clean

CMakeFiles/logic.dir/depend:
	cd /home/kabakov/dev/git/demorobot/stm32/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kabakov/dev/git/demorobot/stm32/test /home/kabakov/dev/git/demorobot/stm32/test /home/kabakov/dev/git/demorobot/stm32/test/build /home/kabakov/dev/git/demorobot/stm32/test/build /home/kabakov/dev/git/demorobot/stm32/test/build/CMakeFiles/logic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/logic.dir/depend

