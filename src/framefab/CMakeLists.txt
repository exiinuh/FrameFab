include_directories(
        SYSTEM
        ${Qt_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${MOSEK_INCLUDE_DIRS}
)

if (NOT APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
endif()

## http://stackoverflow.com/questions/3201154/cmake-automatically-add-all-files-in-a-folder-to-a-target
file(GLOB FRAMEFAB_SRC ${CMAKE_SOURCE_DIR}/src/framefab/*.h ${CMAKE_SOURCE_DIR}/src/framefab/*.cpp)
file(GLOB FRAMEFAB_UI ${CMAKE_SOURCE_DIR}/src/framefab/*.ui)

## http://stackoverflow.com/questions/29968264/linking-and-uic-order-in-a-cmake-qt-project
qt_wrap_ui(FRAMEFAB_UI_H ${FRAMEFAB_UI})

add_executable(framefab ${FRAMEFAB_SRC} ${FRAMEFAB_UI_H})

target_link_libraries(framefab ${MOSEK_LIBRARIES})

