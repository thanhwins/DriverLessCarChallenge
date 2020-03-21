cmake_minimum_required(VERSION 2.8.11)
project(HAL)
set( lib_name HAL)

set(source_files
    Hal.h
    Hal.cpp
    LCDI2C.h
    LCDI2C.cpp

)
add_library( ${lib_name} ${source_files})
