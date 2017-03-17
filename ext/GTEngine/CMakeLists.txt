cmake_minimum_required(VERSION 2.8)
set(BUILD_SHARED_LIBRARY true)

project(GTEngine)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH")

# SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

INCLUDE_DIRECTORIES ( 
	${CMAKE_CURRENT_BINARY_DIR}/Include
	${CMAKE_CURRENT_BINARY_DIR}/src
)

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
		LINKER_LANGUAGE CXX
	)

include_directories(Include)

file(GLOB GTE_ROOT_SRC Source/*.cpp)
file(GLOB GTE_LOWLEVEL_SRC Source/LowLevel/*.cpp)
file(GLOB GTE_MATHEMATICS_SRC  Source/Mathematics/*.cpp)
file(GLOB GTE_IMAGES_SRC  Source/Imagics/*.cpp)
file(GLOB GTE_PHYSICS_SRC  Source/Physics/*.cpp)
file(GLOB GTE_GRAPHICS_SRC  Source/Graphics/*.cpp)
file(GLOB GTE_GRAPHICS_OPENGL_SRC  Source/Graphics/GL4/*.cpp)
file(GLOB GTE_GRAPHICS_GLX_SRC  Source/Graphics/GL4/GLX/*.cpp)
file(GLOB GTE_Applications_SRC  Source/Applications/*.cpp)
file(GLOB GTE_Applications_GLX_SRC  Source/Applications/GLX/*.cpp)

add_library(GTEngine ${GTE_ROOT_SRC} ${GTE_MATHEMATICS_SRC})

set_target_properties(GTEngine PROPERTIES COMPILE_FLAGS -fPIC)
