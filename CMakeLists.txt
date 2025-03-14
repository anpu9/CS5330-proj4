cmake_minimum_required(VERSION 3.22)
project(Proj4)

set(CMAKE_CXX_STANDARD 17)

#  Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find PCL (Point Cloud Library)
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(JsonCpp REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ONNX Runtime setup
set(ONNX_RUNTIME_DIR "/opt/onnxruntime/onnxruntime-osx-arm64-1.20.1")
find_path(ONNX_RUNTIME_INCLUDE_DIR
        NAMES onnxruntime_cxx_api.h
        HINTS ${ONNX_RUNTIME_DIR}/include
        REQUIRED
)
find_library(ONNX_RUNTIME_LIB
        NAMES onnxruntime
        HINTS ${ONNX_RUNTIME_DIR}/lib
        REQUIRED
)

#  Function to add OpenCV-only executables
function(add_opencv_executable exe_name)
    add_executable(${exe_name} ${ARGN})
    target_link_libraries(${exe_name} PRIVATE ${OpenCV_LIBS})
endfunction()

# Function for executables with all dependencies (OpenCV + PCL + ONNX + JsonCpp)
function(add_opencv_depth_pcl_executable exe_name)
    add_executable(${exe_name} ${ARGN})
    target_link_libraries(${exe_name} PRIVATE
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            ${ONNX_RUNTIME_LIB}
            JsonCpp::JsonCpp
    )
    target_include_directories(${exe_name} PRIVATE
            ${OpenCV_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${ONNX_RUNTIME_INCLUDE_DIR}
            JsonCpp::JsonCpp
    )
    target_compile_definitions(${exe_name} PRIVATE ${PCL_DEFINITIONS})
endfunction()

add_opencv_executable(cameraCalibration
        src/vidDisplay.cpp)

add_opencv_executable(augReality
        src/augReality.cpp)

add_opencv_executable(featureDetection
        src/featureDetect.cpp)

add_opencv_depth_pcl_executable(da2Video
        src/da2Video.cpp
        src/pcdUtil.cpp
)

add_opencv_depth_pcl_executable(PointCloud
        src/pointCloudDisplay.cpp
        src/pcdUtil.cpp
)