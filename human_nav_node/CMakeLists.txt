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
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

pkg_check_modules (MOVE3D REQUIRED libmove3d)
pkg_check_modules (GBM REQUIRED gbM)
pkg_check_modules (SOFTMOTION REQUIRED softMotion-libs)

message(STATUS "Move3d Flags: " ${MOVE3D_CFLAGS_OTHER})
message(STATUS "GBM : " ${GBM_LIBRARY_DIRS})
message(STATUS "SOFTMOTION : " ${SOFTMOTION_LIBRARY_DIRS})


include_directories(${MOVE3D_INCLUDE_DIRS} ${GBM_INCLUDE_DIRS} ${SOFTMOTION_INCLUDE_DIRS})
link_directories(${MOVE3D_LIBRARY_DIRS} ${GBM_LIBRARY_DIRS} ${SOFTMOTION_LIBRARY_DIRS})

# cflags for move3d
add_definitions(${MOVE3D_CFLAGS_OTHER})
add_definitions(${GBM_CFLAGS_OTHER})
add_definitions(${SOFTMOTION_CFLAGS_OTHER})
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX -x c++)
    SET (CMAKE_SHARED_LINKER_FLAGS "-dynamiclib -Wl,-headerpad_max_install_names,-undefined,dynamic_lookup")
  ELSE(APPLE)
    ADD_DEFINITIONS(-DLINUX)
  ENDIF(APPLE)
ELSE(UNIX)
 MESSAGE (FATAL_ERROR "Not supported Platform")
ENDIF(UNIX)

rosbuild_add_executable(
	hanp_node src/NavigationNode.cpp 
	src/MotionPlanner.cpp 
	src/codels/mhpMainTaskCodels.cc
	src/codels/mhpNavigationCodels.cc
	src/codels/mhpEnvironmentCodels.cc
	src/codels/mhpInterfaceTaskCodels.cc
	)

target_link_libraries(hanp_node ${MOVE3D_LIBRARIES} ${GBM_LIBRARIES} ${SOFTMOTION_LIBRARIES})
