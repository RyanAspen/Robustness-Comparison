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
CMAKE_SOURCE_DIR = /home/kgabel/collab_orb_slam2/Examples/ROS/compression

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kgabel/collab_orb_slam2/Examples/ROS/compression/build

# Include any dependencies generated for this target.
include CMakeFiles/KittiServerDepth.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/KittiServerDepth.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/KittiServerDepth.dir/flags.make

CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o: CMakeFiles/KittiServerDepth.dir/flags.make
CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o: ../src/compression/KittiServerDepth.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kgabel/collab_orb_slam2/Examples/ROS/compression/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o -c /home/kgabel/collab_orb_slam2/Examples/ROS/compression/src/compression/KittiServerDepth.cpp

CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kgabel/collab_orb_slam2/Examples/ROS/compression/src/compression/KittiServerDepth.cpp > CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.i

CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kgabel/collab_orb_slam2/Examples/ROS/compression/src/compression/KittiServerDepth.cpp -o CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.s

CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o.requires:

.PHONY : CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o.requires

CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o.provides: CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o.requires
	$(MAKE) -f CMakeFiles/KittiServerDepth.dir/build.make CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o.provides.build
.PHONY : CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o.provides

CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o.provides.build: CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o


# Object files for target KittiServerDepth
KittiServerDepth_OBJECTS = \
"CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o"

# External object files for target KittiServerDepth
KittiServerDepth_EXTERNAL_OBJECTS =

../KittiServerDepth: CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o
../KittiServerDepth: CMakeFiles/KittiServerDepth.dir/build.make
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libboost_system.so
../KittiServerDepth: /home/kgabel/Pangolin-0.6/Pangolin-0.6/build/src/libpangolin.so
../KittiServerDepth: ../../../../Thirdparty/DBoW2/lib/libDBoW2.so
../KittiServerDepth: ../../../../Thirdparty/g2o/lib/libg2o.so
../KittiServerDepth: ../../../../lib/libCORB_SLAM2.so
../KittiServerDepth: ../../../../Thirdparty/featureCompression2/install/lib/libLBFC2.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libGLX.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libGLU.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libGLEW.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libEGL.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libX11.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libXext.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libGLX.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libGLU.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libGLEW.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libEGL.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libX11.so
../KittiServerDepth: /usr/lib/x86_64-linux-gnu/libXext.so
../KittiServerDepth: CMakeFiles/KittiServerDepth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kgabel/collab_orb_slam2/Examples/ROS/compression/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../KittiServerDepth"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/KittiServerDepth.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/KittiServerDepth.dir/build: ../KittiServerDepth

.PHONY : CMakeFiles/KittiServerDepth.dir/build

CMakeFiles/KittiServerDepth.dir/requires: CMakeFiles/KittiServerDepth.dir/src/compression/KittiServerDepth.cpp.o.requires

.PHONY : CMakeFiles/KittiServerDepth.dir/requires

CMakeFiles/KittiServerDepth.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/KittiServerDepth.dir/cmake_clean.cmake
.PHONY : CMakeFiles/KittiServerDepth.dir/clean

CMakeFiles/KittiServerDepth.dir/depend:
	cd /home/kgabel/collab_orb_slam2/Examples/ROS/compression/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kgabel/collab_orb_slam2/Examples/ROS/compression /home/kgabel/collab_orb_slam2/Examples/ROS/compression /home/kgabel/collab_orb_slam2/Examples/ROS/compression/build /home/kgabel/collab_orb_slam2/Examples/ROS/compression/build /home/kgabel/collab_orb_slam2/Examples/ROS/compression/build/CMakeFiles/KittiServerDepth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/KittiServerDepth.dir/depend

