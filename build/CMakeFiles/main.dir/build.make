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
CMAKE_SOURCE_DIR = /home/zth/GraduationProject/Exp3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zth/GraduationProject/Exp3/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zth/GraduationProject/Exp3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/main.cpp.o -c /home/zth/GraduationProject/Exp3/main.cpp

CMakeFiles/main.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zth/GraduationProject/Exp3/main.cpp > CMakeFiles/main.dir/main.cpp.i

CMakeFiles/main.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zth/GraduationProject/Exp3/main.cpp -o CMakeFiles/main.dir/main.cpp.s

CMakeFiles/main.dir/src/normal_correction.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/normal_correction.cpp.o: ../src/normal_correction.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zth/GraduationProject/Exp3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/src/normal_correction.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/normal_correction.cpp.o -c /home/zth/GraduationProject/Exp3/src/normal_correction.cpp

CMakeFiles/main.dir/src/normal_correction.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/normal_correction.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zth/GraduationProject/Exp3/src/normal_correction.cpp > CMakeFiles/main.dir/src/normal_correction.cpp.i

CMakeFiles/main.dir/src/normal_correction.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/normal_correction.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zth/GraduationProject/Exp3/src/normal_correction.cpp -o CMakeFiles/main.dir/src/normal_correction.cpp.s

CMakeFiles/main.dir/src/plane_segment.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/plane_segment.cpp.o: ../src/plane_segment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zth/GraduationProject/Exp3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/src/plane_segment.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/plane_segment.cpp.o -c /home/zth/GraduationProject/Exp3/src/plane_segment.cpp

CMakeFiles/main.dir/src/plane_segment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/plane_segment.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zth/GraduationProject/Exp3/src/plane_segment.cpp > CMakeFiles/main.dir/src/plane_segment.cpp.i

CMakeFiles/main.dir/src/plane_segment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/plane_segment.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zth/GraduationProject/Exp3/src/plane_segment.cpp -o CMakeFiles/main.dir/src/plane_segment.cpp.s

CMakeFiles/main.dir/src/pre_process.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/pre_process.cpp.o: ../src/pre_process.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zth/GraduationProject/Exp3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/src/pre_process.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/pre_process.cpp.o -c /home/zth/GraduationProject/Exp3/src/pre_process.cpp

CMakeFiles/main.dir/src/pre_process.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/pre_process.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zth/GraduationProject/Exp3/src/pre_process.cpp > CMakeFiles/main.dir/src/pre_process.cpp.i

CMakeFiles/main.dir/src/pre_process.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/pre_process.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zth/GraduationProject/Exp3/src/pre_process.cpp -o CMakeFiles/main.dir/src/pre_process.cpp.s

CMakeFiles/main.dir/src/visualization.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/visualization.cpp.o: ../src/visualization.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zth/GraduationProject/Exp3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/main.dir/src/visualization.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/visualization.cpp.o -c /home/zth/GraduationProject/Exp3/src/visualization.cpp

CMakeFiles/main.dir/src/visualization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/visualization.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zth/GraduationProject/Exp3/src/visualization.cpp > CMakeFiles/main.dir/src/visualization.cpp.i

CMakeFiles/main.dir/src/visualization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/visualization.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zth/GraduationProject/Exp3/src/visualization.cpp -o CMakeFiles/main.dir/src/visualization.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/main.cpp.o" \
"CMakeFiles/main.dir/src/normal_correction.cpp.o" \
"CMakeFiles/main.dir/src/plane_segment.cpp.o" \
"CMakeFiles/main.dir/src/pre_process.cpp.o" \
"CMakeFiles/main.dir/src/visualization.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/main.cpp.o
main: CMakeFiles/main.dir/src/normal_correction.cpp.o
main: CMakeFiles/main.dir/src/plane_segment.cpp.o
main: CMakeFiles/main.dir/src/pre_process.cpp.o
main: CMakeFiles/main.dir/src/visualization.cpp.o
main: CMakeFiles/main.dir/build.make
main: /usr/local/lib/libopencv_stitching.so.4.5.4
main: /usr/local/lib/libopencv_alphamat.so.4.5.4
main: /usr/local/lib/libopencv_aruco.so.4.5.4
main: /usr/local/lib/libopencv_barcode.so.4.5.4
main: /usr/local/lib/libopencv_bgsegm.so.4.5.4
main: /usr/local/lib/libopencv_bioinspired.so.4.5.4
main: /usr/local/lib/libopencv_ccalib.so.4.5.4
main: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.4
main: /usr/local/lib/libopencv_dnn_superres.so.4.5.4
main: /usr/local/lib/libopencv_dpm.so.4.5.4
main: /usr/local/lib/libopencv_face.so.4.5.4
main: /usr/local/lib/libopencv_freetype.so.4.5.4
main: /usr/local/lib/libopencv_fuzzy.so.4.5.4
main: /usr/local/lib/libopencv_hdf.so.4.5.4
main: /usr/local/lib/libopencv_hfs.so.4.5.4
main: /usr/local/lib/libopencv_img_hash.so.4.5.4
main: /usr/local/lib/libopencv_intensity_transform.so.4.5.4
main: /usr/local/lib/libopencv_line_descriptor.so.4.5.4
main: /usr/local/lib/libopencv_mcc.so.4.5.4
main: /usr/local/lib/libopencv_quality.so.4.5.4
main: /usr/local/lib/libopencv_rapid.so.4.5.4
main: /usr/local/lib/libopencv_reg.so.4.5.4
main: /usr/local/lib/libopencv_rgbd.so.4.5.4
main: /usr/local/lib/libopencv_saliency.so.4.5.4
main: /usr/local/lib/libopencv_stereo.so.4.5.4
main: /usr/local/lib/libopencv_structured_light.so.4.5.4
main: /usr/local/lib/libopencv_superres.so.4.5.4
main: /usr/local/lib/libopencv_surface_matching.so.4.5.4
main: /usr/local/lib/libopencv_tracking.so.4.5.4
main: /usr/local/lib/libopencv_videostab.so.4.5.4
main: /usr/local/lib/libopencv_viz.so.4.5.4
main: /usr/local/lib/libopencv_wechat_qrcode.so.4.5.4
main: /usr/local/lib/libopencv_xfeatures2d.so.4.5.4
main: /usr/local/lib/libopencv_xobjdetect.so.4.5.4
main: /usr/local/lib/libopencv_xphoto.so.4.5.4
main: /usr/lib/libpcl_apps.so
main: /usr/lib/libpcl_cuda_io.so
main: /usr/lib/libpcl_cuda_features.so
main: /usr/lib/libpcl_cuda_segmentation.so
main: /usr/lib/libpcl_cuda_sample_consensus.so
main: /usr/lib/libpcl_outofcore.so
main: /usr/lib/libpcl_gpu_features.so
main: /usr/lib/libpcl_gpu_kinfu.so
main: /usr/lib/libpcl_gpu_kinfu_large_scale.so
main: /usr/lib/libpcl_gpu_people.so
main: /usr/lib/libpcl_gpu_segmentation.so
main: /usr/lib/libpcl_gpu_surface.so
main: /usr/lib/libpcl_people.so
main: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
main: /usr/lib/libOpenNI.so
main: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
main: /usr/lib/libOpenNI2.so
main: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
main: /usr/lib/x86_64-linux-gnu/libfreetype.so
main: /usr/lib/x86_64-linux-gnu/libz.so
main: /usr/lib/x86_64-linux-gnu/libjpeg.so
main: /usr/lib/x86_64-linux-gnu/libpng.so
main: /usr/lib/x86_64-linux-gnu/libtiff.so
main: /usr/lib/x86_64-linux-gnu/libexpat.so
main: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
main: /usr/lib/x86_64-linux-gnu/libqhull_r.so
main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
main: /usr/local/lib/libopencv_shape.so.4.5.4
main: /usr/local/lib/libopencv_highgui.so.4.5.4
main: /usr/local/lib/libopencv_datasets.so.4.5.4
main: /usr/local/lib/libopencv_plot.so.4.5.4
main: /usr/local/lib/libopencv_text.so.4.5.4
main: /usr/local/lib/libopencv_ml.so.4.5.4
main: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.4
main: /usr/local/lib/libopencv_optflow.so.4.5.4
main: /usr/local/lib/libopencv_ximgproc.so.4.5.4
main: /usr/local/lib/libopencv_video.so.4.5.4
main: /usr/local/lib/libopencv_videoio.so.4.5.4
main: /usr/local/lib/libopencv_imgcodecs.so.4.5.4
main: /usr/local/lib/libopencv_objdetect.so.4.5.4
main: /usr/local/lib/libopencv_calib3d.so.4.5.4
main: /usr/local/lib/libopencv_dnn.so.4.5.4
main: /usr/local/lib/libopencv_features2d.so.4.5.4
main: /usr/local/lib/libopencv_flann.so.4.5.4
main: /usr/local/lib/libopencv_photo.so.4.5.4
main: /usr/local/lib/libopencv_imgproc.so.4.5.4
main: /usr/local/lib/libopencv_core.so.4.5.4
main: /usr/lib/libpcl_keypoints.so
main: /usr/lib/libpcl_tracking.so
main: /usr/lib/libpcl_recognition.so
main: /usr/lib/libpcl_registration.so
main: /usr/lib/libpcl_stereo.so
main: /usr/lib/libpcl_surface.so
main: /usr/lib/libpcl_gpu_octree.so
main: /usr/lib/libpcl_gpu_utils.so
main: /usr/lib/libpcl_gpu_containers.so
main: /usr/lib/libpcl_segmentation.so
main: /usr/lib/libpcl_features.so
main: /usr/lib/libpcl_filters.so
main: /usr/lib/libpcl_sample_consensus.so
main: /usr/lib/libpcl_ml.so
main: /usr/lib/libpcl_visualization.so
main: /usr/lib/libpcl_search.so
main: /usr/lib/libpcl_kdtree.so
main: /usr/lib/libpcl_io.so
main: /usr/lib/libpcl_octree.so
main: /usr/lib/libOpenNI.so
main: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
main: /usr/lib/libOpenNI2.so
main: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libjpeg.so
main: /usr/lib/x86_64-linux-gnu/libpng.so
main: /usr/lib/x86_64-linux-gnu/libtiff.so
main: /usr/lib/x86_64-linux-gnu/libexpat.so
main: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libfreetype.so
main: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
main: /usr/lib/x86_64-linux-gnu/libz.so
main: /usr/lib/x86_64-linux-gnu/libGLEW.so
main: /usr/lib/x86_64-linux-gnu/libSM.so
main: /usr/lib/x86_64-linux-gnu/libICE.so
main: /usr/lib/x86_64-linux-gnu/libX11.so
main: /usr/lib/x86_64-linux-gnu/libXext.so
main: /usr/lib/x86_64-linux-gnu/libXt.so
main: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
main: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
main: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
main: /usr/lib/libpcl_common.so
main: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
main: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zth/GraduationProject/Exp3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/zth/GraduationProject/Exp3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zth/GraduationProject/Exp3 /home/zth/GraduationProject/Exp3 /home/zth/GraduationProject/Exp3/build /home/zth/GraduationProject/Exp3/build /home/zth/GraduationProject/Exp3/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

