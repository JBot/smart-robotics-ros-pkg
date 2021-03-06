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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(command_manager src/command_manager.cpp)
rosbuild_add_executable(nestor_voice src/nestor_voice.cpp)
#rosbuild_add_executable(nestor_twit src/nestor_twit.cpp)
rosbuild_add_executable(nestor_weather src/nestor_weather.cpp)
rosbuild_add_executable(nestor_calendar src/nestor_calendar.cpp)
rosbuild_add_executable(nestor_radio src/nestor_radio.cpp)
rosbuild_add_executable(light_manager src/light_manager.cpp)
rosbuild_add_executable(light_detector src/light_detector.cpp)
rosbuild_add_executable(heat_manager src/heat_manager.cpp)
