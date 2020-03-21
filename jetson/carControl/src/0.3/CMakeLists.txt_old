cmake_minimum_required(VERSION 2.8)

project( CarControl-0.3 )

find_package( OpenCV REQUIRED )
set( OPENNI2_INCLUDE /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Include)
set( OPENNI2_REDIST  /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Redist)
include_directories( ${OPENNI2_INCLUDE} )

link_directories ( ${OPENNI2_REDIST} )


if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
               STRINGS "Debug" "Release")
endif()
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

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

include_directories(
      ${OpenCV_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}/stereo_vision
      ${CMAKE_CURRENT_SOURCE_DIR}/lane_detection
      ${CMAKE_CURRENT_SOURCE_DIR}/lane_detection/msac
      ${CMAKE_CURRENT_SOURCE_DIR}/peripheral_driver/i2c
      ${CMAKE_CURRENT_SOURCE_DIR}/peripheral_driver/uart
      ${CMAKE_CURRENT_SOURCE_DIR}/ObjectDetection
      ${CMAKE_CURRENT_SOURCE_DIR}/ObjectRecognition 
      ${CMAKE_CURRENT_SOURCE_DIR}/HAL	
      ${CMAKE_CURRENT_SOURCE_DIR}/multilane
      ${CMAKE_CURRENT_SOURCE_DIR}/include
	     ${CMAKE_CURRENT_SOURCE_DIR}/sign_detection
      ${CMAKE_CURRENT_SOURCE_DIR}/sign_recognize
      ${CMAKE_CURRENT_SOURCE_DIR}/radon
      ${CMAKE_CURRENT_SOURCE_DIR}/extract_info
	${CMAKE_CURRENT_SOURCE_DIR}/openni2
)

link_directories( ${LIB_FREENECT_INSTALL_DIR}/lib
                  ${LIBRARY_OUTPUT_PATH}
)

find_package(OpenMP)
if(OPENMP_FOUND)
    message("Find OpenMP: ${OpenMP_C_FLAGS}, ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(lane_detection)
add_subdirectory(peripheral_driver)
add_subdirectory(stereo_vision)
add_subdirectory(ObjectRecognition)
add_subdirectory(ObjectDetection) 
add_subdirectory(HAL)
add_subdirectory(multilane)
add_subdirectory(sign_detection)
add_subdirectory(sign_recognize)
add_subdirectory(radon)
add_subdirectory(extract_info)
add_subdirectory(openni2)

set( exec_name1 test-steering-coef)
set( source_files1
    main_pid/test-steering-coef.cpp
)
add_executable( ${exec_name1} ${source_files1} )
target_link_libraries( ${exec_name1} i2c-pwm uart HAL)


set( exec_name2 test-autocar)
set( source_files2
    main_pid/automaticCar.cpp
    # if you write extra libraries for automaticCar.cpp, declare them here.
    # See CMakeLists.txt in sub-directory for more details. Eg: 
    # main_pid/depth.h
    # main_pid/depth.cpp
)
add_executable( ${exec_name2} ${source_files2} )
target_link_libraries( ${exec_name2} vanishing-point i2c-pwm uart kinect-cv2 OpenNI2 HAL multilane ${OpenCV_LIBS})


set( exec_name5 testSignRecognize)
set( source_files5
    main_pid/testSignRecognition.cpp
)

add_executable(${exec_name5} ${source_files5})
target_link_libraries(${exec_name5} OpenNI2 ObjectRecognition ObjectDetection ${OpenCV_LIBS})
set( vision_exe vision-test)
set( vision_src main_pid/vision_test.cpp)
add_executable( ${vision_exe} ${vision_src})
target_link_libraries( ${vision_exe} OpenNI2 openni2 i2c-pwm multilane HAL SignDetection signsRecognizer radon extractInfo)
