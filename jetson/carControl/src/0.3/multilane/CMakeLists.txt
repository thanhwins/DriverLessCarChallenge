cmake_minimum_required(VERSION 2.8)

project( OpenCV-Kinect )
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package( OpenCV REQUIRED )
set( OPENNI2_INCLUDE /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Include)
set( OPENNI2_REDIST  /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Redist)
include_directories( ${OPENNI2_INCLUDE} )

link_directories ( ${OPENNI2_REDIST} )
find_package(OpenMP)
if(OPENMP_FOUND)
    message("Find OpenMP: ${OpenMP_C_FLAGS}, ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package( OpenCV REQUIRED )

set(libname multilane)

set(source_files
    multilane.cpp
    multilane.h
)

add_library( ${libname} ${source_files})

set(test_exec_name test-multilane)

add_executable( ${test_exec_name} test_multilane.cpp)

target_link_libraries(${test_exec_name} ${libname} ${OpenCV_LIBS} OpenNI2 )
