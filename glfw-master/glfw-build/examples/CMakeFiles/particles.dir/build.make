# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/e/software/glfw-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/e/software/glfw-master/glfw-build

# Include any dependencies generated for this target.
include examples/CMakeFiles/particles.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/particles.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/particles.dir/flags.make

examples/CMakeFiles/particles.dir/particles.c.o: examples/CMakeFiles/particles.dir/flags.make
examples/CMakeFiles/particles.dir/particles.c.o: ../examples/particles.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/e/software/glfw-master/glfw-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/CMakeFiles/particles.dir/particles.c.o"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/particles.dir/particles.c.o   -c /home/e/software/glfw-master/examples/particles.c

examples/CMakeFiles/particles.dir/particles.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/particles.dir/particles.c.i"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/e/software/glfw-master/examples/particles.c > CMakeFiles/particles.dir/particles.c.i

examples/CMakeFiles/particles.dir/particles.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/particles.dir/particles.c.s"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/e/software/glfw-master/examples/particles.c -o CMakeFiles/particles.dir/particles.c.s

examples/CMakeFiles/particles.dir/particles.c.o.requires:

.PHONY : examples/CMakeFiles/particles.dir/particles.c.o.requires

examples/CMakeFiles/particles.dir/particles.c.o.provides: examples/CMakeFiles/particles.dir/particles.c.o.requires
	$(MAKE) -f examples/CMakeFiles/particles.dir/build.make examples/CMakeFiles/particles.dir/particles.c.o.provides.build
.PHONY : examples/CMakeFiles/particles.dir/particles.c.o.provides

examples/CMakeFiles/particles.dir/particles.c.o.provides.build: examples/CMakeFiles/particles.dir/particles.c.o


examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o: examples/CMakeFiles/particles.dir/flags.make
examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o: ../deps/tinycthread.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/e/software/glfw-master/glfw-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/particles.dir/__/deps/tinycthread.c.o   -c /home/e/software/glfw-master/deps/tinycthread.c

examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/particles.dir/__/deps/tinycthread.c.i"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/e/software/glfw-master/deps/tinycthread.c > CMakeFiles/particles.dir/__/deps/tinycthread.c.i

examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/particles.dir/__/deps/tinycthread.c.s"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/e/software/glfw-master/deps/tinycthread.c -o CMakeFiles/particles.dir/__/deps/tinycthread.c.s

examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o.requires:

.PHONY : examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o.requires

examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o.provides: examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o.requires
	$(MAKE) -f examples/CMakeFiles/particles.dir/build.make examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o.provides.build
.PHONY : examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o.provides

examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o.provides.build: examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o


examples/CMakeFiles/particles.dir/__/deps/getopt.c.o: examples/CMakeFiles/particles.dir/flags.make
examples/CMakeFiles/particles.dir/__/deps/getopt.c.o: ../deps/getopt.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/e/software/glfw-master/glfw-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object examples/CMakeFiles/particles.dir/__/deps/getopt.c.o"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/particles.dir/__/deps/getopt.c.o   -c /home/e/software/glfw-master/deps/getopt.c

examples/CMakeFiles/particles.dir/__/deps/getopt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/particles.dir/__/deps/getopt.c.i"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/e/software/glfw-master/deps/getopt.c > CMakeFiles/particles.dir/__/deps/getopt.c.i

examples/CMakeFiles/particles.dir/__/deps/getopt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/particles.dir/__/deps/getopt.c.s"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/e/software/glfw-master/deps/getopt.c -o CMakeFiles/particles.dir/__/deps/getopt.c.s

examples/CMakeFiles/particles.dir/__/deps/getopt.c.o.requires:

.PHONY : examples/CMakeFiles/particles.dir/__/deps/getopt.c.o.requires

examples/CMakeFiles/particles.dir/__/deps/getopt.c.o.provides: examples/CMakeFiles/particles.dir/__/deps/getopt.c.o.requires
	$(MAKE) -f examples/CMakeFiles/particles.dir/build.make examples/CMakeFiles/particles.dir/__/deps/getopt.c.o.provides.build
.PHONY : examples/CMakeFiles/particles.dir/__/deps/getopt.c.o.provides

examples/CMakeFiles/particles.dir/__/deps/getopt.c.o.provides.build: examples/CMakeFiles/particles.dir/__/deps/getopt.c.o


examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o: examples/CMakeFiles/particles.dir/flags.make
examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o: ../deps/glad_gl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/e/software/glfw-master/glfw-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/particles.dir/__/deps/glad_gl.c.o   -c /home/e/software/glfw-master/deps/glad_gl.c

examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/particles.dir/__/deps/glad_gl.c.i"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/e/software/glfw-master/deps/glad_gl.c > CMakeFiles/particles.dir/__/deps/glad_gl.c.i

examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/particles.dir/__/deps/glad_gl.c.s"
	cd /home/e/software/glfw-master/glfw-build/examples && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/e/software/glfw-master/deps/glad_gl.c -o CMakeFiles/particles.dir/__/deps/glad_gl.c.s

examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o.requires:

.PHONY : examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o.requires

examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o.provides: examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o.requires
	$(MAKE) -f examples/CMakeFiles/particles.dir/build.make examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o.provides.build
.PHONY : examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o.provides

examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o.provides.build: examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o


# Object files for target particles
particles_OBJECTS = \
"CMakeFiles/particles.dir/particles.c.o" \
"CMakeFiles/particles.dir/__/deps/tinycthread.c.o" \
"CMakeFiles/particles.dir/__/deps/getopt.c.o" \
"CMakeFiles/particles.dir/__/deps/glad_gl.c.o"

# External object files for target particles
particles_EXTERNAL_OBJECTS =

examples/particles: examples/CMakeFiles/particles.dir/particles.c.o
examples/particles: examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o
examples/particles: examples/CMakeFiles/particles.dir/__/deps/getopt.c.o
examples/particles: examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o
examples/particles: examples/CMakeFiles/particles.dir/build.make
examples/particles: src/libglfw3.a
examples/particles: /usr/lib/x86_64-linux-gnu/libm.so
examples/particles: /usr/lib/x86_64-linux-gnu/librt.so
examples/particles: /usr/lib/x86_64-linux-gnu/libX11.so
examples/particles: examples/CMakeFiles/particles.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/e/software/glfw-master/glfw-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C executable particles"
	cd /home/e/software/glfw-master/glfw-build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/particles.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/particles.dir/build: examples/particles

.PHONY : examples/CMakeFiles/particles.dir/build

examples/CMakeFiles/particles.dir/requires: examples/CMakeFiles/particles.dir/particles.c.o.requires
examples/CMakeFiles/particles.dir/requires: examples/CMakeFiles/particles.dir/__/deps/tinycthread.c.o.requires
examples/CMakeFiles/particles.dir/requires: examples/CMakeFiles/particles.dir/__/deps/getopt.c.o.requires
examples/CMakeFiles/particles.dir/requires: examples/CMakeFiles/particles.dir/__/deps/glad_gl.c.o.requires

.PHONY : examples/CMakeFiles/particles.dir/requires

examples/CMakeFiles/particles.dir/clean:
	cd /home/e/software/glfw-master/glfw-build/examples && $(CMAKE_COMMAND) -P CMakeFiles/particles.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/particles.dir/clean

examples/CMakeFiles/particles.dir/depend:
	cd /home/e/software/glfw-master/glfw-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/e/software/glfw-master /home/e/software/glfw-master/examples /home/e/software/glfw-master/glfw-build /home/e/software/glfw-master/glfw-build/examples /home/e/software/glfw-master/glfw-build/examples/CMakeFiles/particles.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/particles.dir/depend

