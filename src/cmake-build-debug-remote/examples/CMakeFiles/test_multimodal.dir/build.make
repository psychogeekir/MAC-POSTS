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
CMAKE_SOURCE_DIR = /home/qiling/Documents/MAC-POSTS/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote

# Include any dependencies generated for this target.
include examples/CMakeFiles/test_multimodal.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/test_multimodal.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/test_multimodal.dir/flags.make

examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o: examples/CMakeFiles/test_multimodal.dir/flags.make
examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o: ../examples/test_multimodal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/examples && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/examples/test_multimodal.cpp

examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_multimodal.dir/test_multimodal.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/examples && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/examples/test_multimodal.cpp > CMakeFiles/test_multimodal.dir/test_multimodal.cpp.i

examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_multimodal.dir/test_multimodal.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/examples && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/examples/test_multimodal.cpp -o CMakeFiles/test_multimodal.dir/test_multimodal.cpp.s

examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o.requires:

.PHONY : examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o.requires

examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o.provides: examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/test_multimodal.dir/build.make examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o.provides.build
.PHONY : examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o.provides

examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o.provides.build: examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o


# Object files for target test_multimodal
test_multimodal_OBJECTS = \
"CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o"

# External object files for target test_multimodal
test_multimodal_EXTERNAL_OBJECTS =

examples/test_multimodal: examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o
examples/test_multimodal: examples/CMakeFiles/test_multimodal.dir/build.make
examples/test_multimodal: minami/libminami.so
examples/test_multimodal: 3rdparty/adv_ds/libadv_ds.so
examples/test_multimodal: snap-core/libSnap.so
examples/test_multimodal: glib-core/libGlib.so
examples/test_multimodal: 3rdparty/g3log/libg3log.so
examples/test_multimodal: examples/CMakeFiles/test_multimodal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_multimodal"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_multimodal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/test_multimodal.dir/build: examples/test_multimodal

.PHONY : examples/CMakeFiles/test_multimodal.dir/build

examples/CMakeFiles/test_multimodal.dir/requires: examples/CMakeFiles/test_multimodal.dir/test_multimodal.cpp.o.requires

.PHONY : examples/CMakeFiles/test_multimodal.dir/requires

examples/CMakeFiles/test_multimodal.dir/clean:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/examples && $(CMAKE_COMMAND) -P CMakeFiles/test_multimodal.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/test_multimodal.dir/clean

examples/CMakeFiles/test_multimodal.dir/depend:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qiling/Documents/MAC-POSTS/src /home/qiling/Documents/MAC-POSTS/src/examples /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/examples /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/examples/CMakeFiles/test_multimodal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/test_multimodal.dir/depend

