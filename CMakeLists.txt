#cmake_minimum_required(VERSION 2.8.8)
cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0012 NEW)
project(e_robot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_PREFIX_PATH "/opt/Qt5.5.0/5.5/gcc_64/")

#This line is very important!!!
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

 add_compile_options(-std=c++11)
#SET(CMAKE_C_COMPILER g++)
#SET(CMAK_CXX_COMPILER g++)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()


# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)


# ${QT_INCLUDE_DIRS} ${QT_LIBRARIES} and so on are all Qt4 Macro Definitions!!!!!
# Find the QtWidgets library



find_package(Qt5  REQUIRED COMPONENTS Core Widgets Gui Network Multimedia WebKitWidgets WebKit SerialPort )
set(QT_LIBRARIES Qt5::Widgets Qt5::Core  Qt5::Gui Qt5::Network Qt5::Multimedia Qt5::WebKitWidgets Qt5::WebKit Qt5::SerialPort)

# 3-party
find_package(OpenCV 3.4.6 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED )
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )


message( [qt] ${Qt5Core_INCLUDE_DIRS} )
message( [qt] ${Qt5Core_LIBRARIES} )
message( [qt] ${Qt5Widgets_INCLDE_DIRS} )
message( [qt] ${Qt5Widgets_LIBRARIES} )
message( [qt] ${Qt5Multimedia_INCLUDE_DIRS} )
message( [qt] ${Qt5Multimedia_LIBRARIES} )
message( [qt] ${Qt5SerialPort_INCLUDE_DIRS} )


#include dirs
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)
link_directories(${OpenCV_LIB_DIR})

#realsense d435
#include_directories(include ${realsense_INCLUDE_DIR})
include_directories(include ${realsense_INCLUDE_DIR}
            ./src/rgbd_camera/third-party/imgui
            ./src/rgbd_camera/third-party)

#GLUT
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})

#OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})

# cpp files
#aux_source_directory(src SRC_LIST)

# ui files
#qt5_wrap_ui(ui_FILES src/mainwindow.ui)
# resource files
#qt5_add_resources(qrc_FILES resources/resources.qrc)

#add_executable(${PROJECT_NAME} ${SRC_LIST} ${ui_FILES} ${qrc_FILES} ) #${qrc_FILES}
#qt5_use_modules(${PROJECT_NAME} Core Widgets Gui)
#target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBRARIES}  ${EIGEN3_LIBS} ${realsense2_LIBRARY})


file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h
    mono_camera/*.h socket_robot/*.h rgbd_camera/*.h global_camera/*.h serial_common/*.h remote_terminal/*.h
    watch_dog/*.h ping_check/*.h )#added *.h

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC}) #change QT4 to QT5

message( [Main] ${PROJECT_NAME} ) # ./main.cpp./mainwindow.cpp
message( [Main] ${QT_RESOURCES_CPP} )# build/qrc_resource.cppmessage( [Main] ${QT_FORMS_HPP} )# build/qrc_resource.cpp
message( [Main] ${QT_MOC} )

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp
    mono_camera/*.cpp socket_robot/*.cpp rgbd_camera/*.cpp global_camera/*.cpp serial_common/*.cpp remote_terminal/*.cpp
    watch_dog/*.cpp ping_check/*.cpp)

SET(SRC ./src/rgbd_camera/example.h
        ./src/rgbd_camera/example.cpp
#        ./src/rgbd_camera/Show3D.h
#       ./src/rgbd_camera/Show3D.cpp
        ./src/rgbd_camera/third-party/imgui/imgui.cpp
        ./src/rgbd_camera/third-party/imgui/imgui_draw.cpp
        ./src/rgbd_camera/third-party/imgui/imgui_impl_glfw.cpp
        )

add_executable(${PROJECT_NAME}
                ${QT_SOURCES}
                ${QT_RESOURCES_CPP}
                ${QT_FORMS_HPP}
                ${QT_MOC}
                ${SRC})

#qt5_use_modules(${PROJECT_NAME} Core Widgets Gui)
target_link_libraries(${PROJECT_NAME}
        ${QT_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBS}
        ${realsense2_LIBRARY}
        ${CERES_LIBRARIES}
	${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        ${GLUT_glut_LIBRARY})


#GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(./glfw-master REQUIRED)
include_directories(glfw-master)
include_directories(glfw-master/deps)
include_directories(glfw-master/include)

set(GLAD "${GLFW_SOURCE_DIR}/deps/glad/glad.h"
        "${GLFW_SOURCE_DIR}/deps/glad.c")
set(GETOPT "${GLFW_SOURCE_DIR}/deps/getopt.h"
        "${GLFW_SOURCE_DIR}/deps/getopt.c")
set(TINYCTHREAD "${GLFW_SOURCE_DIR}/deps/tinycthread.h"
        "${GLFW_SOURCE_DIR}/deps/tinycthread.c")

target_link_libraries(${PROJECT_NAME} glfw ${GLFW_LIBRARIES})











