cmake_minimum_required(VERSION 3.0.0)

find_package( OpenGL REQUIRED )
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLU REQUIRED)
include_directories(${GLU_INCLUDE_DIR})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

file(GLOB poly2tri
 poly2tri/common/*.cc
 poly2tri/sweep/*.cc
 )

add_library(${PROJECT_NAME}
 SHARED
 ${poly2tri}
 )


add_executable(testbed
 testbed/main.cc
 )

target_link_libraries(testbed
 ${PROJECT_NAME}
 -lglfw
 -lGL -lGLU -lGLEW
 )
