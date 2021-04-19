cmake_minimum_required(VERSION 3.2)
project(robot_basic_tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "C++14 support: OK")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ----- Set compiler options ----
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 ")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -Wall -Wno-sign-compare")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall -Wno-sign-compare")


# ----- color defined -----
if (NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(ColourBold "${Esc}[1m")
    set(Red "${Esc}[31m")
    set(Green "${Esc}[32m")
    set(Yellow "${Esc}[33m")
    set(Blue "${Esc}[34m")
    set(Magenta "${Esc}[35m")
    set(Cyan "${Esc}[36m")
    set(White "${Esc}[37m")
    set(BoldRed "${Esc}[1;31m")
    set(BoldGreen "${Esc}[1;32m")
    set(BoldYellow "${Esc}[1;33m")
    set(BoldBlue "${Esc}[1;34m")
    set(BoldMagenta "${Esc}[1;35m")
    set(BoldCyan "${Esc}[1;36m")
    set(BoldWhite "${Esc}[1;37m")
endif ()

# ----- Set Version -----
# 程序版本以及编译信息
include(${PROJECT_SOURCE_DIR}/cmake/getGitInfo.cmake)
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")
message("${Green}-- Build timestamp is ${BUILD_TIMESTAMP} ${ColourReset}")

# 获取分支名
set(GIT_BRANCH "")
get_git_branch(GIT_BRANCH)
message("${Green}-- Git branch is ${GIT_BRANCH} ${ColourReset}")

# 获取HASH
set(GIT_HASH "")
get_git_hash(GIT_HASH)
message("${Green}-- Git hash is ${GIT_HASH} ${ColourReset}")

set(USE_VERSION_INFO 1)
if (USE_VERSION_INFO)
    set(VERSION_MAJOR 0)
    set(VERSION_MINOR 0)
    set(VERSION_PATCH 1)
    message("${Green}-- Version is ${VERSION_MAJOR} ${VERSION_MINOR} ${VERSION_PATCH}" ${ColourReset})
endif ()

configure_file(
        "${PROJECT_SOURCE_DIR}/include/version_info.h.in"
        "${PROJECT_SOURCE_DIR}/include/version_info.h"
)

# ----- Set debug -----
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(USE_STACK_TRACE_LOGGER ON CACHE BOOL "Enable automatic stack trace logger of google-glog" FORCE)
if (USE_STACK_TRACE_LOGGER)
    message(STATUS "Stack trace logger: ENABLED")
    # Glog
    find_package(Glog REQUIRED)
    add_definitions(-D USE_STACK_TRACE_LOGGER)
    message(STATUS "GLOG_INCLUDE_DIR: ${GLOG_INCLUDE_DIR}")
    message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")

else ()
    message(STATUS "Stack trace logger: DISABLED")
endif ()

# ----- ros相关 -----
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        message_filters
        sensor_msgs
        )
#message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
#message(STATUS "catkin_LIBRARIES: ${catkin_LIBRARIES}")

# ----- 设置生成thirdpart .so的路径 -----
set(RDPARTY_SO_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/lib)
message(STATUS "RDPARTY_SO_PATH: ${RDPARTY_SO_PATH}")

# 设置依赖库地址
#set(OpenCV_LIB_DIR "/home/anson/work/lib/opencv43/lib/cmake/opencv4/" CACHE STRING "The directory containing a CMake configuration file for OpenCV." FORCE)
set(OpenCV_LIB_DIR "/home/anson/work/lib/" CACHE STRING "The directory containing a CMake configuration file for OpenCV.")
message(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

#set(Ceres_LIB_DIR "/home/anson/work/lib/ceres-solver-2.0.0/lib/cmake/Ceres/" CACHE STRING "The Path for CMake to find Ceres library")
set(Ceres_LIB_DIR "/home/anson/work/lib/" CACHE STRING "The directory containing a CMake configuration file for Ceres.")
message(STATUS "Ceres_LIB_DIR: ${Ceres_LIB_DIR}")

# ----- Build thirdparty -----
message("${Magenta}-------------- build thirdparty lib begin ----------${ColourReset}")
message(STATUS "enter thirdparty/camera_model")
add_subdirectory(thirdparty/camera_model)
message(STATUS "exit thirdparty/camera_model")
message(STATUS "enter thirdparty/cv_bridge")
add_subdirectory(thirdparty/cv_bridge_rbt)
message(STATUS "exit thirdparty/cv_bridge")
message("${Magenta}-------------- build thirdparty lib end ------------${ColourReset}")


# ----- find package -----
# GLM
find_package(GLM REQUIRED)

if (POLICY CMP0022)
    cmake_policy(SET CMP0022 NEW)
endif ()

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE GLVND)
    # set(OpenGL_GL_PREFERENCE LEGACY)
endif ()

# openGL
find_package(OpenGL REQUIRED)
message(STATUS "OPENGL_LIBRARIES: ${OPENGL_LIBRARIES}")

# pcl >= 1.8
find_package(PCL 1.8 REQUIRED
        COMPONENTS
        common
        io
        )

#message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
#message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# boost
#find_package(Boost REQUIRED COMPONENTS
#        system
#        thread)

# OpenCV
find_package(OpenCV 4.3 QUIET
        COMPONENTS
        core
        imgproc
        highgui
        )
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4.3 PATHS ${OpenCV_LIB_DIR} NO_DEFAULT_PATH REQUIRED
            COMPONENTS
            core
            imgproc
            highgui
            )
endif ()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")


# Ceres
find_package(Ceres 2.0 QUIET)
if (NOT Ceres_FOUND)
    find_package(Ceres PATHS ${Ceres_LIB_DIR} NO_DEFAULT_PATH REQUIRED)
endif ()

message(STATUS "CERES_VERSION: ${CERES_VERSION}")
message(STATUS "CERES_LIBRARIES: ${CERES_LIBRARIES}")

# MRPT
#find_package(MRPT 2.2 COMPONENTS math)
#message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
#message(STATUS "MRPT_LIBRARIES: ${MRPT_LIBRARIES}")

# ----- catking config -----
catkin_package(
)

# ----- Build -----

# build target imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)
add_library(imgui SHARED
        thirdparty/gl3w/gl3w.c
        thirdparty/imgui/imgui.cpp
        thirdparty/imgui/imgui_demo.cpp
        thirdparty/imgui/imgui_draw.cpp
        thirdparty/imgui/imgui_widgets.cpp
        thirdparty/imgui/imgui_tables.cpp
        thirdparty/imgui/backends/imgui_impl_glfw.cpp
        thirdparty/imgui/backends/imgui_impl_opengl3.cpp
        )

set_target_properties(imgui PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${RDPARTY_SO_PATH}
        )

target_include_directories(imgui PUBLIC
        $<BUILD_INTERFACE:thirdparty/gl3w;${GLM_INCLUDE_DIRS};thirdparty/imgui>
        )

target_link_libraries(imgui
        PUBLIC
        ${OPENGL_LIBRARIES}
        glfw
        )

# build target  guik
add_library(guik SHARED
        src/glk/mesh.cpp
        src/glk/simple_lines.cpp
        src/glk/lines.cpp
        src/glk/colormap.cpp
        src/glk/glsl_shader.cpp
        src/glk/frame_buffer.cpp
        src/glk/pointcloud_buffer.cpp
        src/glk/primitives/primitives.cpp
        src/glk/loaders/miniply.cpp
        src/glk/loaders/ply_loader.cpp
        src/guik/gl_canvas.cpp
        src/guik/camera_control.cpp
        src/guik/projection_control.cpp
        src/guik/imgui_application.cpp
        )

set_target_properties(guik PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${RDPARTY_SO_PATH}
        )

target_include_directories(guik PUBLIC
        $<BUILD_INTERFACE:./include;${PCL_INCLUDE_DIRS}>
        )

target_link_libraries(guik
        PUBLIC
        imgui
        ${PCL_LIBRARIES}
        )

# build target algorithm
add_library(algorithm SHARED
        src/algorithm/random/RandomGenerator.cpp
        src/algorithm/ransac/ransac_applications.cpp
        src/algorithm/util.cpp
        src/algorithm/line_detector.cpp
        src/algorithm/line_fitting.cpp
        src/algorithm/laser_cam_ceres.cpp
        src/algorithm/two_lasers_ceres.cpp
        src/algorithm/two_cameras_ceres.cpp
        )

set_target_properties(algorithm PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${RDPARTY_SO_PATH}
        )

target_include_directories(algorithm PUBLIC
        $<BUILD_INTERFACE:./include;${OpenCV_INCLUDE_DIRS};${catkin_INCLUDE_DIRS}>
        )

target_link_libraries(algorithm
        PUBLIC
        ${OpenCV_LIBRARIES}
        # ${MRPT_LIBRARIES}
        ${CERES_LIBRARIES}
        camera_model
        )

# build target
add_executable(${PROJECT_NAME}
        src/dev/sensor.cpp
        src/dev/camera.cpp
        src/dev/laser.cpp
        src/dev/sensor_manager.cpp
        src/dev/image_show.cpp
        src/dev/april_board.cpp
        src/dev/util.cpp
        src/calibration/camera_laser_calib.cpp
        src/calibration/two_lasers_calib.cpp
        src/calibration/two_cameras_calib.cpp
        src/util/image_loader.cpp
        src/robot_basic_tools.cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIR}
        thirdparty/json
        thirdparty/portable-file-dialogs
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        cv_bridge_rbt
        guik
        algorithm
        glog
        )
