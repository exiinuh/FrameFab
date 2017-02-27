include_directories(
  SYSTEM
  ${Qt_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${MOSEK_INCLUDE_DIRS}
  ${GTENGINE_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/ext
)

 #if (NOT APPLE)
 #   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
 #endif()

# include Qt basic make functions
include(QtCommon)

add_project_meta(META_FILES_TO_INCLUDE)

set(RESOURCES_FILES mainwindow.qrc)

# https://cmake.org/Wiki/CMake:How_To_Find_Libraries
#find_package(Qt REQUIRED QtCore QtWidgets QtGui QtOpenGL)
#find_package(Qt COMPONENTS QtCore QtGui QtOpenGL QtWidgets REQUIRED)
#find_package(OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

## http://stackoverflow.com/questions/3201154/cmake-automatically-add-all-files-in-a-folder-to-a-target
file(GLOB FRAMEFAB_SRC ${CMAKE_SOURCE_DIR}/src/framefab/*.h ${CMAKE_SOURCE_DIR}/src/framefab/*.cpp)
file(GLOB FRAMEFAB_UI ${CMAKE_SOURCE_DIR}/src/framefab/*.ui)

## http://stackoverflow.com/questions/29968264/linking-and-uic-order-in-a-cmake-qt-project
QT5_WRAP_UI(FRAMEFAB_UI_H ${FRAMEFAB_UI})

add_executable(framefab ${FRAMEFAB_SRC} ${FRAMEFAB_UI_H})

target_link_libraries(framefab ${MOSEK_LIBRARIES} GTEngine)

qt5_use_modules(${PROJECT_NAME} Widgets)

