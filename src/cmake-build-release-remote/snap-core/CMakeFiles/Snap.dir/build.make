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
CMAKE_BINARY_DIR = /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote

# Include any dependencies generated for this target.
include snap-core/CMakeFiles/Snap.dir/depend.make

# Include the progress variables for this target.
include snap-core/CMakeFiles/Snap.dir/progress.make

# Include the compile flags for this target's objects.
include snap-core/CMakeFiles/Snap.dir/flags.make

snap-core/CMakeFiles/Snap.dir/Snap.cpp.o: snap-core/CMakeFiles/Snap.dir/flags.make
snap-core/CMakeFiles/Snap.dir/Snap.cpp.o: ../snap-core/Snap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object snap-core/CMakeFiles/Snap.dir/Snap.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/snap-core && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Snap.dir/Snap.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/snap-core/Snap.cpp

snap-core/CMakeFiles/Snap.dir/Snap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Snap.dir/Snap.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/snap-core && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/snap-core/Snap.cpp > CMakeFiles/Snap.dir/Snap.cpp.i

snap-core/CMakeFiles/Snap.dir/Snap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Snap.dir/Snap.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/snap-core && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/snap-core/Snap.cpp -o CMakeFiles/Snap.dir/Snap.cpp.s

snap-core/CMakeFiles/Snap.dir/Snap.cpp.o.requires:

.PHONY : snap-core/CMakeFiles/Snap.dir/Snap.cpp.o.requires

snap-core/CMakeFiles/Snap.dir/Snap.cpp.o.provides: snap-core/CMakeFiles/Snap.dir/Snap.cpp.o.requires
	$(MAKE) -f snap-core/CMakeFiles/Snap.dir/build.make snap-core/CMakeFiles/Snap.dir/Snap.cpp.o.provides.build
.PHONY : snap-core/CMakeFiles/Snap.dir/Snap.cpp.o.provides

snap-core/CMakeFiles/Snap.dir/Snap.cpp.o.provides.build: snap-core/CMakeFiles/Snap.dir/Snap.cpp.o


# Object files for target Snap
Snap_OBJECTS = \
"CMakeFiles/Snap.dir/Snap.cpp.o"

# External object files for target Snap
Snap_EXTERNAL_OBJECTS =

snap-core/libSnap.so: snap-core/CMakeFiles/Snap.dir/Snap.cpp.o
snap-core/libSnap.so: snap-core/CMakeFiles/Snap.dir/build.make
snap-core/libSnap.so: glib-core/libGlib.so
snap-core/libSnap.so: snap-core/CMakeFiles/Snap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libSnap.so"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/snap-core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Snap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
snap-core/CMakeFiles/Snap.dir/build: snap-core/libSnap.so

.PHONY : snap-core/CMakeFiles/Snap.dir/build

snap-core/CMakeFiles/Snap.dir/requires: snap-core/CMakeFiles/Snap.dir/Snap.cpp.o.requires

.PHONY : snap-core/CMakeFiles/Snap.dir/requires

snap-core/CMakeFiles/Snap.dir/clean:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/snap-core && $(CMAKE_COMMAND) -P CMakeFiles/Snap.dir/cmake_clean.cmake
.PHONY : snap-core/CMakeFiles/Snap.dir/clean

snap-core/CMakeFiles/Snap.dir/depend:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qiling/Documents/MAC-POSTS/src /home/qiling/Documents/MAC-POSTS/src/snap-core /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/snap-core /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/snap-core/CMakeFiles/Snap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : snap-core/CMakeFiles/Snap.dir/depend
