cmake_minimum_required(VERSION 3.0)
project(framefab)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH")

# add folder for findModule and qt make supportive functions 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# http://stackoverflow.com/questions/3371127/use-isystem-instead-of-i-with-cmake
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

# http://cmake.3232098.n2.nabble.com/Default-value-for-CMAKE-BUILD-TYPE-td7550756.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to ' Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
            "MinSizeRel" "RelWithDebInfo")
endif()

#--- Find Eigen
find_package(Eigen REQUIRED)

#--- Find MOSEK
find_package(MOSEK REQUIRED)

#--- Find LAPACK
find_package(LAPACK REQUIRED)

#--- Find GL & GLUT
find_package(GL REQUIRED)
find_package(GLUT REQUIRED)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(GTENGINE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/GTEngine/Include)

add_subdirectory(ext)
add_subdirectory(src)
