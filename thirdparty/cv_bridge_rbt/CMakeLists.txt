project(cv_bridge_rbt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Boost 1.58 REQUIRED
        COMPONENTS
        iostreams)
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
set(OpenCV_LIB_DIR "/home/moi/include/opencv4.5/lib/" CACHE STRING "The directory containing a CMake configuration file for OpenCV.")
message(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

# OpenCV
find_package(OpenCV 4.0 QUIET
        COMPONENTS
        core
        imgproc
        calib3d
        highgui
        )
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4.0 PATHS ${OpenCV_LIB_DIR} NO_DEFAULT_PATH REQUIRED
            COMPONENTS
            core
            imgproc
            calib3d
            highgui
            )
endif ()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

find_package(catkin REQUIRED
        COMPONENTS
        rosconsole
        sensor_msgs)

# add library
add_library(${PROJECT_NAME} SHARED
        src/cv_bridge.cpp
        src/rgb_colors.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${RDPARTY_SO_PATH}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>;${catkin_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        )


