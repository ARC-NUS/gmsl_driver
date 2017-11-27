cmake_minimum_required(VERSION 3.1)
project(gmsl_n_cameras)

set (CMAKE_CXX_STANDARD 11)

# FindDriveworks.cmake, ArchConfiguration.cmake, and LibFindMacros.cmake were needed for my setup they are taken from driveworks/samples/cmake/
# ArchConfiguration.cmake was the only file that needed small changes, remove the fatal error on line 17 and add the following lines in its place
         set(VIBRANTE TRUE)
         add_definitions(-DVIBRANTE)
# this is the path I placed the driveworks cmake files in
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)
find_package(GLEW REQUIRED)
find_package(Driveworks REQUIRED)
find_package(OpenCV REQUIRED)
#  NO_DEFAULT_PATH)
#NO_CMAKE_ENVIRONMENT_PATH
#NO_CMAKE_PATH
#NO_SYSTEM_ENVIRONMENT_PATH)
#find_package(PkgConfig REQUIRED)
#pkg_search_module(glfw required glfw3)


# SET THESE VARIABLES TO YOUR DRIVEWORKS LOCATIONS
set(Driveworks_INCLUDE_DIR /usr/local/driveworks-0.3/include/)
set(Driveworks_LIBRARY /usr/local/driveworks-0.3/lib/)


# set flags for CUDA nvcc
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -DMY_DEF=1") 
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMY_DEF=1" )
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMY_DEF=1" ) 
set(LIB_TYPE STATIC)
CUDA_ADD_LIBRARY(Uploadlib ${LIB_TYPE} src/upload.cu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  cv_bridge 
  image_transport
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)
link_directories(
  ${Driveworks_LIBRARY})


MESSAGE("Driveworks include directory ${Driveworks_INCLUDE_DIR}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Driveworks_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${GLEW_INCLUDE_DIR}
)

# TODO: add a FindNvmedia.cmake file for this? Why does it not exist?
include_directories(
 /usr/share/visionworks/sources/3rdparty/nvmedia/
 /usr/share/visionworks/VisionWorks-1.6-Samples/3rdparty/glfw3/include/
 /usr/local/driveworks/samples/3rdparty/linux-aarch64/glfw-3.1.1/include
)


# ros node name template
set(NODE_NAME ${PROJECT_NAME}_node)

#CUDA compile
cuda_compile(MAIN_O src/main.cu)
#cuda_add_executable(${NODE_NAME} src/main.cu)
#target_link_libraries( ${catkin_LIBRARIES} ) 
#add_dependencies(${NODE_NAME} ${PROJECT_NAME}_generate_messages_cpp ${LINE_PROFILER_CU_O} )



add_executable(${NODE_NAME} src/main.cpp
  src/ProgramArguments.cpp
  src/ConsoleColor.cpp
  src/WindowEGL.cpp
  src/WindowGLFW.cpp  
  src/cv_connection.cpp
  src/ResourceManager.cpp
  src/SampleFramework.cpp
  src/Camera.cpp
)

add_dependencies(${NODE_NAME} Uploadlib)

target_link_libraries(${NODE_NAME}
  ${catkin_LIBRARIES}
  nvmedia  
  ${GLEW_LIBRARY}
  driveworks
  GLESv2
  EGL
  drm
#  glfw3
  ${CUDA_LIBRARY}
#  ${OpenCV_LIBS}
  opencv_cudaimgproc
  Uploadlib
)
