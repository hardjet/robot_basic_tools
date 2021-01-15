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
# include(${PROJECT_SOURCE_DIR}/cmake/getGitInfo.cmake)
# string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")
# message("Build timestamp is ${BUILD_TIMESTAMP}")

# set(USE_VERSION_INFO 0)
# if (USE_VERSION_INFO)
#     set(VERSION_MAJOR 0)
#     set(VERSION_MINOR 2)
#     set(VERSION_PATCH 1)
#     message("Version is ${VERSION_MAJOR} ${VERSION_MINOR} ${VERSION_PATCH}")
# endif ()

# configure_file(
#         "${PROJECT_SOURCE_DIR}/include/version_info.h.in"
#         "${PROJECT_SOURCE_DIR}/include/version_info.h"
# )

# ----- Set debug -----

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(USE_STACK_TRACE_LOGGER OFF CACHE BOOL "Enable automatic stack trace logger of google-glog")
set(USE_STACK_TRACE_LOGGER OFF)
if(USE_STACK_TRACE_LOGGER)
    message(STATUS "Stack trace logger: ENABLED")
    # Glog
    find_package(Glog REQUIRED)
    include_directories(${GLOG_INCLUDE_DIR})
    add_definitions(-D USE_STACK_TRACE_LOGGER)
    message(STATUS "GLOG_INCLUDE_DIR: ${GLOG_INCLUDE_DIR}")
    message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")

else()
    message(STATUS "Stack trace logger: DISABLED")
endif()

# ----- find package -----
find_package(GLM REQUIRED)

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE GLVND)
#    set(OpenGL_GL_PREFERENCE LEGACY)
endif ()

find_package(OpenGL REQUIRED)

# pcl
find_package(PCL 1.8 REQUIRED
        COMPONENTS
        common
        #        kdtree
        #        search
        # filters     # 包含了上面所有的
        )
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")

add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
#message(STATUS "PCL_COMMON_LIBRARIES: ${PCL_COMMON_LIBRARIES}")
#message(STATUS "PCL_KDTREE_LIBRARIES: ${PCL_KDTREE_LIBRARIES}")
#message(STATUS "PCL_SEARCH_LIBRARIES: ${PCL_SEARCH_LIBRARIES}")
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# ros相关
find_package(catkin REQUIRED COMPONENTS
        roscpp
)

##################
# catking config #
##################
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES interactive_map_correction
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

###########
## Build ##
###########
# add_subdirectory(hdl_graph_slam)
# add_subdirectory(thirdparty/ground_filter)
# add_subdirectory(thirdparty/octomap_conversion)

include_directories(
        include
        thirdparty/gl3w
        thirdparty/imgui
        thirdparty/imgui/examples
        thirdparty/portable-file-dialogs
        ${GLM_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)

add_library(imgui STATIC
        thirdparty/gl3w/gl3w.c
        thirdparty/imgui/imgui.cpp
        thirdparty/imgui/imgui_demo.cpp
        thirdparty/imgui/imgui_draw.cpp
        thirdparty/imgui/imgui_widgets.cpp
        thirdparty/imgui/backends/imgui_impl_glfw.cpp
        thirdparty/imgui/backends/imgui_impl_opengl3.cpp
        )
target_link_libraries(imgui
        ${OPENGL_LIBRARIES}
        glfw
        dl
        )

add_library(guik STATIC
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

# add_executable(interactive_slam
#         src/interactive_slam.cpp
#         src/hdl_graph_slam/registration_methods.cpp
#         src/hdl_graph_slam/interactive_graph.cpp
#         src/hdl_graph_slam/interactive_keyframe.cpp
#         src/hdl_graph_slam/version_modal.cpp
#         src/hdl_graph_slam/graph_edit_window.cpp
#         src/hdl_graph_slam/edge_refinement_window.cpp
#         src/hdl_graph_slam/plane_detection_window.cpp
#         src/hdl_graph_slam/plane_alignment_modal.cpp
#         src/hdl_graph_slam/manual_loop_close_modal.cpp
#         src/hdl_graph_slam/automatic_loop_close_window.cpp
#         src/hdl_graph_slam/scan_context_manager.cpp
#         src/hdl_graph_slam/manual_adjust_merge_modal.cpp
#         src/hdl_graph_slam/view/edge_view.cpp
#         src/hdl_graph_slam/view/vertex_view.cpp
#         src/hdl_graph_slam/map_block_io.cpp
#         src/hdl_graph_slam/map_grid.cpp)

# target_link_libraries(interactive_slam
#         ${OPENGL_LIBRARIES}
#         ${catkin_LIBRARIES}
#         glfw
#         guik
#         imgui
#         hdl_graph_slam
#         ground_filter
#         octomap_conversion
#         glog
#         )

# file(COPY data DESTINATION .)
