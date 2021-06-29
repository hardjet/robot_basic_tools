project(camera_model)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC -fopenmp -Wno-all")
set(OpenCV_LIB_DIR "/home/moi/include/opencv4.5/lib/" CACHE STRING "The directory containing a CMake configuration file for OpenCV.")
message(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

#set(Ceres_LIB_DIR "/home/anson/work/lib/ceres-solver-2.0.0/lib/cmake/Ceres/" CACHE STRING "The Path for CMake to find Ceres library")
set(Ceres_LIB_DIR "/home/moi/include/ceres/lib/" CACHE STRING "The directory containing a CMake configuration file for Ceres.")
message(STATUS "Ceres_LIB_DIR: ${Ceres_LIB_DIR}")
find_package(Boost REQUIRED COMPONENTS
        filesystem
        program_options
        system)
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")

# OpenCV
find_package(OpenCV QUIET
        COMPONENTS
        core
        imgproc
        calib3d
        highgui
        )
if (NOT OpenCV_FOUND)
    find_package(OpenCV PATHS ${OpenCV_LIB_DIR} NO_DEFAULT_PATH REQUIRED
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

# Ceres
find_package(Ceres 2.0 QUIET)
if (NOT Ceres_FOUND)
    find_package(Ceres PATHS ${Ceres_LIB_DIR} NO_DEFAULT_PATH REQUIRED)
endif ()

aux_source_directory(./src/apriltag AprilTagSrc)
aux_source_directory(./src/apriltag_frontend AprilTagSrc2)

set(CAMERA_MODEL_FILES
        ./src/camera_models/Camera.cc
        ./src/camera_models/CostFunctionFactory.cc
        ./src/camera_models/CameraFactory.cc
        ./src/camera_models/PinholeCamera.cc
        ./src/camera_models/PinholeFullCamera.cc
        ./src/camera_models/CataCamera.cc
        ./src/camera_models/EquidistantCamera.cc
        ./src/camera_models/ScaramuzzaCamera.cc
        ./src/camera_models/PolyFisheyeCamera.cpp
        ./src/camera_models/SplineCamera.cpp
        ./src/camera_models/FovCamera.cpp
        ./src/sparse_graph/Transform.cc
        ./src/code_utils/math_utils/Polynomial.cpp
        ./src/code_utils/cv_utils.cc
        ./src/gpl/EigenQuaternionParameterization.cc
        ./src/gpl/gpl.cc
        ${AprilTagSrc}
        ${AprilTagSrc2}
        )

add_library(${PROJECT_NAME} SHARED
        src/chessboard/Chessboard.cc
        src/calib/CameraCalibration.cc
        ${CAMERA_MODEL_FILES}
        )

set_target_properties(${PROJECT_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${RDPARTY_SO_PATH}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        )


