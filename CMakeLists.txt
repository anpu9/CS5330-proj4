cmake_minimum_required(VERSION 3.22)
project(Proj4)

set(CMAKE_CXX_STANDARD 17)

#  Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#  Function to add OpenCV-only executables
function(add_opencv_executable exe_name)
    add_executable(${exe_name} ${ARGN})
    target_link_libraries(${exe_name} PRIVATE ${OpenCV_LIBS})
endfunction()

add_opencv_executable(VidDisplay
        src/vidDisplay.cpp)

add_opencv_executable(augReality
        src/augReality.cpp)
