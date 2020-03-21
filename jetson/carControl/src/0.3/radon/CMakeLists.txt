cmake_minimum_required(VERSION 2.8)

project( radon )
IF(MSVC)
  ADD_DEFINITIONS(-DNOMINMAX)
  # Another MSVC quirk. http://stackoverflow.com/questions/6832666/lnk2019-when-including-asio-headers-solution-generated-with-cmake
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP") # For multi-threaded compilation on MSVC
ELSE()
  IF(APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++") # C++11 support on Apple clang
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# C++11 support on GCC
  ENDIF()
ENDIF()
find_package( OpenCV REQUIRED )

set( source_files
	radon.h
	radon.cpp
)

set( lib_name radon)
add_library( ${lib_name} ${source_files} )
target_link_libraries( ${lib_name} ${OpenCV_LIBS} vanishing-point)


#set( test_exec_name test_radon)
#add_executable( ${test_exec_name}  test_radon.cpp )

#target_link_libraries( ${test_exec_name} ${lib_name} ${OpenCV_LIBS})
