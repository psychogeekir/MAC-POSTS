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
include 3rdparty/g3log/CMakeFiles/g3log.dir/depend.make

# Include the progress variables for this target.
include 3rdparty/g3log/CMakeFiles/g3log.dir/progress.make

# Include the compile flags for this target's objects.
include 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make

3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o: 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make
3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o: ../3rdparty/g3log/filesink.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object 3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g3log.dir/filesink.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/filesink.cpp

3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g3log.dir/filesink.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/filesink.cpp > CMakeFiles/g3log.dir/filesink.cpp.i

3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g3log.dir/filesink.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/filesink.cpp -o CMakeFiles/g3log.dir/filesink.cpp.s

3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o.requires:

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o.requires

3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o.provides: 3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o.requires
	$(MAKE) -f 3rdparty/g3log/CMakeFiles/g3log.dir/build.make 3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o.provides.build
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o.provides

3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o.provides.build: 3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o


3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o: 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make
3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o: ../3rdparty/g3log/logmessage.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object 3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g3log.dir/logmessage.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logmessage.cpp

3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g3log.dir/logmessage.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logmessage.cpp > CMakeFiles/g3log.dir/logmessage.cpp.i

3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g3log.dir/logmessage.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logmessage.cpp -o CMakeFiles/g3log.dir/logmessage.cpp.s

3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o.requires:

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o.requires

3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o.provides: 3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o.requires
	$(MAKE) -f 3rdparty/g3log/CMakeFiles/g3log.dir/build.make 3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o.provides.build
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o.provides

3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o.provides.build: 3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o


3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o: 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make
3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o: ../3rdparty/g3log/logcapture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object 3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g3log.dir/logcapture.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logcapture.cpp

3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g3log.dir/logcapture.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logcapture.cpp > CMakeFiles/g3log.dir/logcapture.cpp.i

3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g3log.dir/logcapture.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logcapture.cpp -o CMakeFiles/g3log.dir/logcapture.cpp.s

3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o.requires:

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o.requires

3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o.provides: 3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o.requires
	$(MAKE) -f 3rdparty/g3log/CMakeFiles/g3log.dir/build.make 3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o.provides.build
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o.provides

3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o.provides.build: 3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o


3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o: 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make
3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o: ../3rdparty/g3log/loglevels.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object 3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g3log.dir/loglevels.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/loglevels.cpp

3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g3log.dir/loglevels.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/loglevels.cpp > CMakeFiles/g3log.dir/loglevels.cpp.i

3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g3log.dir/loglevels.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/loglevels.cpp -o CMakeFiles/g3log.dir/loglevels.cpp.s

3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o.requires:

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o.requires

3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o.provides: 3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o.requires
	$(MAKE) -f 3rdparty/g3log/CMakeFiles/g3log.dir/build.make 3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o.provides.build
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o.provides

3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o.provides.build: 3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o


3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o: 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make
3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o: ../3rdparty/g3log/logworker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object 3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g3log.dir/logworker.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logworker.cpp

3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g3log.dir/logworker.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logworker.cpp > CMakeFiles/g3log.dir/logworker.cpp.i

3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g3log.dir/logworker.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/logworker.cpp -o CMakeFiles/g3log.dir/logworker.cpp.s

3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o.requires:

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o.requires

3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o.provides: 3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o.requires
	$(MAKE) -f 3rdparty/g3log/CMakeFiles/g3log.dir/build.make 3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o.provides.build
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o.provides

3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o.provides.build: 3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o


3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o: 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make
3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o: ../3rdparty/g3log/crashhandler_unix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object 3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g3log.dir/crashhandler_unix.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/crashhandler_unix.cpp

3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g3log.dir/crashhandler_unix.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/crashhandler_unix.cpp > CMakeFiles/g3log.dir/crashhandler_unix.cpp.i

3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g3log.dir/crashhandler_unix.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/crashhandler_unix.cpp -o CMakeFiles/g3log.dir/crashhandler_unix.cpp.s

3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o.requires:

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o.requires

3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o.provides: 3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o.requires
	$(MAKE) -f 3rdparty/g3log/CMakeFiles/g3log.dir/build.make 3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o.provides.build
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o.provides

3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o.provides.build: 3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o


3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o: 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make
3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o: ../3rdparty/g3log/time.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object 3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g3log.dir/time.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/time.cpp

3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g3log.dir/time.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/time.cpp > CMakeFiles/g3log.dir/time.cpp.i

3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g3log.dir/time.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/time.cpp -o CMakeFiles/g3log.dir/time.cpp.s

3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o.requires:

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o.requires

3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o.provides: 3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o.requires
	$(MAKE) -f 3rdparty/g3log/CMakeFiles/g3log.dir/build.make 3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o.provides.build
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o.provides

3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o.provides.build: 3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o


3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o: 3rdparty/g3log/CMakeFiles/g3log.dir/flags.make
3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o: ../3rdparty/g3log/g3log.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object 3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g3log.dir/g3log.cpp.o -c /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/g3log.cpp

3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g3log.dir/g3log.cpp.i"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/g3log.cpp > CMakeFiles/g3log.dir/g3log.cpp.i

3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g3log.dir/g3log.cpp.s"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log/g3log.cpp -o CMakeFiles/g3log.dir/g3log.cpp.s

3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o.requires:

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o.requires

3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o.provides: 3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o.requires
	$(MAKE) -f 3rdparty/g3log/CMakeFiles/g3log.dir/build.make 3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o.provides.build
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o.provides

3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o.provides.build: 3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o


# Object files for target g3log
g3log_OBJECTS = \
"CMakeFiles/g3log.dir/filesink.cpp.o" \
"CMakeFiles/g3log.dir/logmessage.cpp.o" \
"CMakeFiles/g3log.dir/logcapture.cpp.o" \
"CMakeFiles/g3log.dir/loglevels.cpp.o" \
"CMakeFiles/g3log.dir/logworker.cpp.o" \
"CMakeFiles/g3log.dir/crashhandler_unix.cpp.o" \
"CMakeFiles/g3log.dir/time.cpp.o" \
"CMakeFiles/g3log.dir/g3log.cpp.o"

# External object files for target g3log
g3log_EXTERNAL_OBJECTS =

3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/build.make
3rdparty/g3log/libg3log.so: 3rdparty/g3log/CMakeFiles/g3log.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX shared library libg3log.so"
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/g3log.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
3rdparty/g3log/CMakeFiles/g3log.dir/build: 3rdparty/g3log/libg3log.so

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/build

3rdparty/g3log/CMakeFiles/g3log.dir/requires: 3rdparty/g3log/CMakeFiles/g3log.dir/filesink.cpp.o.requires
3rdparty/g3log/CMakeFiles/g3log.dir/requires: 3rdparty/g3log/CMakeFiles/g3log.dir/logmessage.cpp.o.requires
3rdparty/g3log/CMakeFiles/g3log.dir/requires: 3rdparty/g3log/CMakeFiles/g3log.dir/logcapture.cpp.o.requires
3rdparty/g3log/CMakeFiles/g3log.dir/requires: 3rdparty/g3log/CMakeFiles/g3log.dir/loglevels.cpp.o.requires
3rdparty/g3log/CMakeFiles/g3log.dir/requires: 3rdparty/g3log/CMakeFiles/g3log.dir/logworker.cpp.o.requires
3rdparty/g3log/CMakeFiles/g3log.dir/requires: 3rdparty/g3log/CMakeFiles/g3log.dir/crashhandler_unix.cpp.o.requires
3rdparty/g3log/CMakeFiles/g3log.dir/requires: 3rdparty/g3log/CMakeFiles/g3log.dir/time.cpp.o.requires
3rdparty/g3log/CMakeFiles/g3log.dir/requires: 3rdparty/g3log/CMakeFiles/g3log.dir/g3log.cpp.o.requires

.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/requires

3rdparty/g3log/CMakeFiles/g3log.dir/clean:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log && $(CMAKE_COMMAND) -P CMakeFiles/g3log.dir/cmake_clean.cmake
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/clean

3rdparty/g3log/CMakeFiles/g3log.dir/depend:
	cd /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qiling/Documents/MAC-POSTS/src /home/qiling/Documents/MAC-POSTS/src/3rdparty/g3log /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log /home/qiling/Documents/MAC-POSTS/src/cmake-build-release-remote/3rdparty/g3log/CMakeFiles/g3log.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : 3rdparty/g3log/CMakeFiles/g3log.dir/depend

