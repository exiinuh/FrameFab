cmake_minimum_required(VERSION 2.8.11)
project(framefab)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# http://stackoverflow.com/questions/3371127/use-isystem-instead-of-i-with-cmake
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_AUTOMOC ON)

# http://cmake.3232098.n2.nabble.com/Default-value-for-CMAKE-BUILD-TYPE-td7550756.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
            "MinSizeRel" "RelWithDebInfo")
endif()

# https://cmake.org/Wiki/CMake:How_To_Find_Libraries
find_package(Qt COMPONENTS QtCore QtGui QtOpenGL QtWidgets REQUIRED)
#find_package(OpenGL REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

#find_package()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(GTENGINE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/GTEngine/Include)

add_subdirectory(src)
# add_subdirectory(ext)