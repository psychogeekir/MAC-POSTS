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
include pybinder/CMakeFiles/MNMAPI.dir/depend.make

# Include the progress variables for this target.
include pybinder/CMakeFiles/MNMAPI.dir/progress.make

# Include the compile flags for this target's objects.
include pybinder/CMakeFiles/MNMAPI.dir/flags.make

pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o: pybinder/CMakeFiles/MNMAPI.dir/flags.make
pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o: ../pybinder/src/dta_api.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/pybinder/src/dta_api.cpp

pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MNMAPI.dir/src/dta_api.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/pybinder/src/dta_api.cpp > CMakeFiles/MNMAPI.dir/src/dta_api.cpp.i

pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MNMAPI.dir/src/dta_api.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/pybinder/src/dta_api.cpp -o CMakeFiles/MNMAPI.dir/src/dta_api.cpp.s

pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o.requires:

.PHONY : pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o.requires

pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o.provides: pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o.requires
	$(MAKE) -f pybinder/CMakeFiles/MNMAPI.dir/build.make pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o.provides.build
.PHONY : pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o.provides

pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o.provides.build: pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o


pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o: pybinder/CMakeFiles/MNMAPI.dir/flags.make
pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o: ../pybinder/src/typecast_ground\ .cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o -c "/home/qiling/Documents/MAC-POSTS/src/pybinder/src/typecast_ground .cpp"

pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/qiling/Documents/MAC-POSTS/src/pybinder/src/typecast_ground .cpp" > CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.i

pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/qiling/Documents/MAC-POSTS/src/pybinder/src/typecast_ground .cpp" -o CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.s

pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o.requires:

.PHONY : pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o.requires

pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o.provides: pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o.requires
	$(MAKE) -f pybinder/CMakeFiles/MNMAPI.dir/build.make pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o.provides.build
.PHONY : pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o.provides

pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o.provides.build: pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o


# Object files for target MNMAPI
MNMAPI_OBJECTS = \
"CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o" \
"CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o"

# External object files for target MNMAPI
MNMAPI_EXTERNAL_OBJECTS =

pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o
pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o
pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: pybinder/CMakeFiles/MNMAPI.dir/build.make
pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: minami/libminami.so
pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: snap-core/libSnap.so
pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: glib-core/libGlib.so
pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: 3rdparty/g3log/libg3log.so
pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: 3rdparty/adv_ds/libadv_ds.so
pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so: pybinder/CMakeFiles/MNMAPI.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module MNMAPI.cpython-36m-x86_64-linux-gnu.so"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MNMAPI.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pybinder/CMakeFiles/MNMAPI.dir/build: pybinder/MNMAPI.cpython-36m-x86_64-linux-gnu.so

.PHONY : pybinder/CMakeFiles/MNMAPI.dir/build

pybinder/CMakeFiles/MNMAPI.dir/requires: pybinder/CMakeFiles/MNMAPI.dir/src/dta_api.cpp.o.requires
pybinder/CMakeFiles/MNMAPI.dir/requires: pybinder/CMakeFiles/MNMAPI.dir/src/typecast_ground_.cpp.o.requires

.PHONY : pybinder/CMakeFiles/MNMAPI.dir/requires

pybinder/CMakeFiles/MNMAPI.dir/clean:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder && $(CMAKE_COMMAND) -P CMakeFiles/MNMAPI.dir/cmake_clean.cmake
.PHONY : pybinder/CMakeFiles/MNMAPI.dir/clean

pybinder/CMakeFiles/MNMAPI.dir/depend:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qiling/Documents/MAC-POSTS/src /home/qiling/Documents/MAC-POSTS/src/pybinder /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder /home/qiling/Documents/MAC-POSTS/src/cmake-build-debug-remote/pybinder/CMakeFiles/MNMAPI.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pybinder/CMakeFiles/MNMAPI.dir/depend
