cmake_minimum_required(VERSION 2.8)

project( extractInfo)
find_package(OpenCV REQUIRED)

set (source_files
	extractInfo.h
	extractInfo.cpp
)

set (lib_name extractInfo)
add_library(${lib_name} ${source_files})
target_link_libraries(${lib_name} ${OpenCV_LIBS} vanishing-point radon)

add_executable(test_extract_info test_extract_info.cpp)
target_link_libraries( test_extract_info ${OpenCV_LIBS} ${lib_name} radon)
