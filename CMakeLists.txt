cmake_minimum_required(VERSION 3.22)
project(Proj4)

set(CMAKE_CXX_STANDARD 17)

#  Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find OpenGL
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLM REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Assimp REQUIRED)
# Include directories
include_directories(${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS})

# Function to add OpenCV + OpenGL executables
function(add_ogl_opencv_executable exe_name)
    add_executable(${exe_name} ${ARGN})
    target_link_libraries(${exe_name} PRIVATE ${OpenCV_LIBS} ${OPENGL_LIBRARIES} GLEW::GLEW glfw ${ASSIMP_LIBRARIES})
endfunction()

#  Function to add OpenCV-only executables
function(add_opencv_executable exe_name)
    add_executable(${exe_name} ${ARGN})
    target_link_libraries(${exe_name} PRIVATE ${OpenCV_LIBS})
endfunction()

add_opencv_executable(VidDisplay
        src/vidDisplay.cpp)

add_ogl_opencv_executable(augReality
        src/augReality.cpp)
