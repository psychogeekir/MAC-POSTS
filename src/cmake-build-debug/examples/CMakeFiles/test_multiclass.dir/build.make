# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

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
CMAKE_COMMAND = /snap/clion/149/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/149/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/qiling/Documents/MAC-POSTS/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug

# Include any dependencies generated for this target.
include examples/CMakeFiles/test_multiclass.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/test_multiclass.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/test_multiclass.dir/flags.make

examples/CMakeFiles/test_multiclass.dir/test_multiclass.cpp.o: examples/CMakeFiles/test_multiclass.dir/flags.make
examples/CMakeFiles/test_multiclass.dir/test_multiclass.cpp.o: ../examples/test_multiclass.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/test_multiclass.dir/test_multiclass.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/examples && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_multiclass.dir/test_multiclass.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/examples/test_multiclass.cpp

examples/CMakeFiles/test_multiclass.dir/test_multiclass.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_multiclass.dir/test_multiclass.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/examples && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/examples/test_multiclass.cpp > CMakeFiles/test_multiclass.dir/test_multiclass.cpp.i

examples/CMakeFiles/test_multiclass.dir/test_multiclass.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_multiclass.dir/test_multiclass.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/examples && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/examples/test_multiclass.cpp -o CMakeFiles/test_multiclass.dir/test_multiclass.cpp.s

# Object files for target test_multiclass
test_multiclass_OBJECTS = \
"CMakeFiles/test_multiclass.dir/test_multiclass.cpp.o"

# External object files for target test_multiclass
test_multiclass_EXTERNAL_OBJECTS =

examples/test_multiclass: examples/CMakeFiles/test_multiclass.dir/test_multiclass.cpp.o
examples/test_multiclass: examples/CMakeFiles/test_multiclass.dir/build.make
examples/test_multiclass: minami/libminami.so
examples/test_multiclass: 3rdparty/adv_ds/libadv_ds.so
examples/test_multiclass: snap-core/libSnap.so
examples/test_multiclass: glib-core/libGlib.so
examples/test_multiclass: 3rdparty/g3log/libg3log.so
examples/test_multiclass: examples/CMakeFiles/test_multiclass.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_multiclass"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_multiclass.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/test_multiclass.dir/build: examples/test_multiclass

.PHONY : examples/CMakeFiles/test_multiclass.dir/build

examples/CMakeFiles/test_multiclass.dir/clean:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/examples && $(CMAKE_COMMAND) -P CMakeFiles/test_multiclass.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/test_multiclass.dir/clean

examples/CMakeFiles/test_multiclass.dir/depend:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qiling/Documents/MAC-POSTS/src /home/qiling/Documents/MAC-POSTS/src/examples /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/examples /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug/examples/CMakeFiles/test_multiclass.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/test_multiclass.dir/depend

