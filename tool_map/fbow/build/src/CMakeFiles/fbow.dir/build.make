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
CMAKE_SOURCE_DIR = /data/colmap/src/tools/fbow

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /data/colmap/src/tools/fbow/build

# Include any dependencies generated for this target.
include src/CMakeFiles/fbow.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/fbow.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/fbow.dir/flags.make

src/CMakeFiles/fbow.dir/fbow.cpp.o: src/CMakeFiles/fbow.dir/flags.make
src/CMakeFiles/fbow.dir/fbow.cpp.o: ../src/fbow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/colmap/src/tools/fbow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/fbow.dir/fbow.cpp.o"
	cd /data/colmap/src/tools/fbow/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fbow.dir/fbow.cpp.o -c /data/colmap/src/tools/fbow/src/fbow.cpp

src/CMakeFiles/fbow.dir/fbow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fbow.dir/fbow.cpp.i"
	cd /data/colmap/src/tools/fbow/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/colmap/src/tools/fbow/src/fbow.cpp > CMakeFiles/fbow.dir/fbow.cpp.i

src/CMakeFiles/fbow.dir/fbow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fbow.dir/fbow.cpp.s"
	cd /data/colmap/src/tools/fbow/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/colmap/src/tools/fbow/src/fbow.cpp -o CMakeFiles/fbow.dir/fbow.cpp.s

src/CMakeFiles/fbow.dir/fbow.cpp.o.requires:

.PHONY : src/CMakeFiles/fbow.dir/fbow.cpp.o.requires

src/CMakeFiles/fbow.dir/fbow.cpp.o.provides: src/CMakeFiles/fbow.dir/fbow.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/fbow.dir/build.make src/CMakeFiles/fbow.dir/fbow.cpp.o.provides.build
.PHONY : src/CMakeFiles/fbow.dir/fbow.cpp.o.provides

src/CMakeFiles/fbow.dir/fbow.cpp.o.provides.build: src/CMakeFiles/fbow.dir/fbow.cpp.o


src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o: src/CMakeFiles/fbow.dir/flags.make
src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o: ../src/vocabulary_creator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/colmap/src/tools/fbow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o"
	cd /data/colmap/src/tools/fbow/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fbow.dir/vocabulary_creator.cpp.o -c /data/colmap/src/tools/fbow/src/vocabulary_creator.cpp

src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fbow.dir/vocabulary_creator.cpp.i"
	cd /data/colmap/src/tools/fbow/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/colmap/src/tools/fbow/src/vocabulary_creator.cpp > CMakeFiles/fbow.dir/vocabulary_creator.cpp.i

src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fbow.dir/vocabulary_creator.cpp.s"
	cd /data/colmap/src/tools/fbow/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/colmap/src/tools/fbow/src/vocabulary_creator.cpp -o CMakeFiles/fbow.dir/vocabulary_creator.cpp.s

src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o.requires:

.PHONY : src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o.requires

src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o.provides: src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/fbow.dir/build.make src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o.provides.build
.PHONY : src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o.provides

src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o.provides.build: src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o


# Object files for target fbow
fbow_OBJECTS = \
"CMakeFiles/fbow.dir/fbow.cpp.o" \
"CMakeFiles/fbow.dir/vocabulary_creator.cpp.o"

# External object files for target fbow
fbow_EXTERNAL_OBJECTS =

src/libfbow.so.0.0.1: src/CMakeFiles/fbow.dir/fbow.cpp.o
src/libfbow.so.0.0.1: src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o
src/libfbow.so.0.0.1: src/CMakeFiles/fbow.dir/build.make
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_dnn.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_ml.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_objdetect.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_shape.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_stitching.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_superres.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_videostab.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_viz.so.3.4.4
src/libfbow.so.0.0.1: /usr/lib/gcc/x86_64-linux-gnu/7/libgomp.so
src/libfbow.so.0.0.1: /usr/lib/x86_64-linux-gnu/libpthread.so
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_calib3d.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_features2d.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_flann.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_highgui.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_photo.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_video.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_videoio.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_imgcodecs.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_imgproc.so.3.4.4
src/libfbow.so.0.0.1: /usr/local/lib/libopencv_core.so.3.4.4
src/libfbow.so.0.0.1: src/CMakeFiles/fbow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/data/colmap/src/tools/fbow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libfbow.so"
	cd /data/colmap/src/tools/fbow/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fbow.dir/link.txt --verbose=$(VERBOSE)
	cd /data/colmap/src/tools/fbow/build/src && $(CMAKE_COMMAND) -E cmake_symlink_library libfbow.so.0.0.1 libfbow.so.0.0 libfbow.so

src/libfbow.so.0.0: src/libfbow.so.0.0.1
	@$(CMAKE_COMMAND) -E touch_nocreate src/libfbow.so.0.0

src/libfbow.so: src/libfbow.so.0.0.1
	@$(CMAKE_COMMAND) -E touch_nocreate src/libfbow.so

# Rule to build all files generated by this target.
src/CMakeFiles/fbow.dir/build: src/libfbow.so

.PHONY : src/CMakeFiles/fbow.dir/build

src/CMakeFiles/fbow.dir/requires: src/CMakeFiles/fbow.dir/fbow.cpp.o.requires
src/CMakeFiles/fbow.dir/requires: src/CMakeFiles/fbow.dir/vocabulary_creator.cpp.o.requires

.PHONY : src/CMakeFiles/fbow.dir/requires

src/CMakeFiles/fbow.dir/clean:
	cd /data/colmap/src/tools/fbow/build/src && $(CMAKE_COMMAND) -P CMakeFiles/fbow.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/fbow.dir/clean

src/CMakeFiles/fbow.dir/depend:
	cd /data/colmap/src/tools/fbow/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /data/colmap/src/tools/fbow /data/colmap/src/tools/fbow/src /data/colmap/src/tools/fbow/build /data/colmap/src/tools/fbow/build/src /data/colmap/src/tools/fbow/build/src/CMakeFiles/fbow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/fbow.dir/depend

