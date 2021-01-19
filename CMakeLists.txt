cmake_minimum_required(VERSION 2.8.3)
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
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")


# ----- Set Version -----
# 程序版本以及编译信息
include(${PROJECT_SOURCE_DIR}/cmake/getGitInfo.cmake)
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")
message("Build timestamp is ${BUILD_TIMESTAMP}")

# 获取分支名
set(GIT_BRANCH "")
get_git_branch(GIT_BRANCH)
message("Git branch is ${GIT_BRANCH}")

# 获取HASH
set(GIT_HASH "")
get_git_hash(GIT_HASH)
message("Git hash is ${GIT_HASH}")

set(USE_VERSION_INFO 1)
if (USE_VERSION_INFO)
    set(VERSION_MAJOR 0)
    set(VERSION_MINOR 0)
    set(VERSION_PATCH 1)
    message("Version is ${VERSION_MAJOR} ${VERSION_MINOR} ${VERSION_PATCH}")
endif ()

configure_file(
        "${PROJECT_SOURCE_DIR}/include/version_info.h.in"
        "${PROJECT_SOURCE_DIR}/include/version_info.h"
)

# ----- Set debug -----

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(USE_STACK_TRACE_LOGGER OFF CACHE BOOL "Enable automatic stack trace logger of google-glog")
set(USE_STACK_TRACE_LOGGER OFF)
if (USE_STACK_TRACE_LOGGER)
    message(STATUS "Stack trace logger: ENABLED")
    # Glog
    find_package(Glog REQUIRED)
    include_directories(${GLOG_INCLUDE_DIR})
    add_definitions(-D USE_STACK_TRACE_LOGGER)
    message(STATUS "GLOG_INCLUDE_DIR: ${GLOG_INCLUDE_DIR}")
    message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")

else ()
    message(STATUS "Stack trace logger: DISABLED")
endif ()

# ----- find package -----
find_package(GLM REQUIRED)

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE GLVND)
    # set(OpenGL_GL_PREFERENCE LEGACY)
endif ()

find_package(OpenGL REQUIRED)
message(STATUS "OPENGL_LIBRARIES: ${OPENGL_LIBRARIES}")

# pcl
find_package(PCL 1.7 REQUIRED
        COMPONENTS
        common
        io
        )
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")

add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
#message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# ros相关
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

# ----- catking config -----
catkin_package(
)

# ----- Build -----
# add_subdirectory(hdl_graph_slam)

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
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
        )

target_include_directories(imgui PUBLIC
        $<BUILD_INTERFACE:thirdparty/gl3w;${GLM_INCLUDE_DIRS};thirdparty/imgui>
        )

target_link_libraries(imgui
        PUBLIC
#        ${OPENGL_LIBRARIES}
        glfw
#        dl
        )

# build target  guik
add_library(guik SHARED
        src/glk/mesh.cpp
        src/glk/lines.cpp
        src/glk/colormap.cpp
        src/glk/glsl_shader.cpp
        src/glk/frame_buffer.cpp
        src/glk/pointcloud_buffer.cpp
        src/glk/primitives/primitives.cpp
        src/glk/loaders/ply_loader.cpp
        src/guik/gl_canvas.cpp
        src/guik/model_control.cpp
        src/guik/camera_control.cpp
        src/guik/projection_control.cpp
        src/guik/imgui_application.cpp
        )

set_target_properties(guik PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/
        )

target_include_directories(guik PUBLIC
        $<BUILD_INTERFACE:include;${PCL_INCLUDE_DIRS}>
        )

target_link_libraries(guik
        PUBLIC
        ${OPENGL_LIBRARIES}
#        ${PCL_LIBRARIES}
#        glfw
#        dl
        imgui
        )

# build target ${PROJECT_NAME}
add_executable(${PROJECT_NAME}
        src/main.cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE
        ${catkin_INCLUDE_DIRS}
        thirdparty/portable-file-dialogs
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        guik
        imgui
        glog
        )

#file(COPY data DESTINATION /home/anson/share)
