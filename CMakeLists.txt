cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(OMPL_DIR "~/opt/omplapp-0.9.1-Source")
include_directories (${OMPL_DIR}/ompl/src)                     #ompl
include_directories (${OMPL_DIR}/src)                          #ompl-app
include_directories (${OMPL_DIR}/assimp-prefix/include/assimp) #assimp

include_directories (${PROJECT_SOURCE_DIR}/include)
file(GLOB_RECURSE PROJECT_SRC src/*.cpp)

rosbuild_add_executable(${PROJECT_NAME} ${PROJECT_SRC})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

target_link_libraries(${PROJECT_NAME} ${OMPL_DIR}/lib/libompl_app.so)