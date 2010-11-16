cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
set(V4l2STEREO_PATH src)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#access to openmp
set(ENABLE_OPENMP 1)
#set(CCMAKE_CXX_FLAGS "-Wall -pedantic -fopenmp")
#set(CCMAKE_C_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "-lgomp -lcvm_ia32")

include_directories(${V4l2STEREO_HOME})
link_directories( ${LINK_DIRECTORIES} "/usr/lib/libcvm;/usr/lib" )

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


#common commands for building c++ executables and libraries
rosbuild_add_library(stereocam_broadcast
    ${V4l2STEREO_PATH}/rectify.cpp
    ${V4l2STEREO_PATH}/camcalib.cpp
    ${V4l2STEREO_PATH}/libcam.cpp
    ${V4l2STEREO_PATH}/stereo.cpp
    ${V4l2STEREO_PATH}/polynomial.cpp
    ${V4l2STEREO_PATH}/elas/triangle.cpp
    ${V4l2STEREO_PATH}/elas/elimination.cpp
    ${V4l2STEREO_PATH}/elas/descriptor.cpp
    ${V4l2STEREO_PATH}/elas/elas.cpp
    ${V4l2STEREO_PATH}/calibration/cvex.cpp
    ${V4l2STEREO_PATH}/calibration/CvexCameraCalibration.cpp
    ${V4l2STEREO_PATH}/calibration/CvexStereoCameraCalibration.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/polynomial.cpp)
#target_link_libraries(stereocam_broadcast ${ELAS_LIBS})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_check_for_sse()
if(HAS_SSE2_EXTENSIONS)
rosbuild_add_compile_flags(stereocam_broadcast ${SSE_FLAGS})
endif(HAS_SSE2_EXTENSIONS)
rosbuild_add_compile_flags(stereocam_broadcast "-g -Wall -pedantic -fopenmp")

rosbuild_add_executable(stereocam_broadcast src/stereocam_broadcast.cpp)
rosbuild_add_executable(stereocam_subscribe src/stereocam_subscribe.cpp)
