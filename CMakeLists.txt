cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CXX_STANDARD 11)


find_package( OpenGL REQUIRED )
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLU REQUIRED)
include_directories(${GLU_INCLUDE_DIR})


file(GLOB poly2tri
 poly2tri/common/*.cc
 poly2tri/sweep/*.cc
 )

add_library(${PROJECT_NAME}
 SHARED
 ${poly2tri}
 )




# PCL

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#set(CMAKE_C_FLAGS“$ {CMAKE_C_FLAGS} -fPIC”)
#set(CMAKE_CXX_FLAGS“$ {CMAKE_CXX_FLAGS} -fPIC”)

#设置工程包含当前目录，非必须
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#设置自动生成moc文件，一定要设置
set(CMAKE_AUTOMOC ON)

set(CMAKE_AUTORCC ON)

#set(CMAKE_PREFIX_PATH /deploy/dist CACHE PATH "" FORCE)

#查找需要的Qt库文件，最好每一个库都要写，Qt也会根据依赖关系自动添加
find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)

find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS log system program_options)

