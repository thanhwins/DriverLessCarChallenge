cmake_minimum_required(VERSION 2.8)

project( ObjectRecognition )

set( lib_name ObjectRecognition)

set(source_files
    SignProc.h
    SignProc.cpp
    SignRecognition.cpp
    SignRecognition.h

)

add_library( ${lib_name} ${source_files})
