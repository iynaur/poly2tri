cmake_minimum_required(VERSION 3.0.0)

project(poly2tri)
set(CMAKE_CXX_STANDARD 11)

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")



find_package( OpenGL REQUIRED )
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLU REQUIRED)
include_directories(${GLU_INCLUDE_DIR})




#查找需要的Qt库文件，最好每一个库都要写，Qt也会根据依赖关系自动添加
find_package(Qt5 COMPONENTS
             Core
             Widgets
             Gui
             OpenGL)

find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)
ADD_DEFINITIONS (${QT_DEFINITIONS})

set(QT_USE_QTMAIN TRUE)

file(GLOB poly2tri
 poly2tri/common/*.cc
 poly2tri/sweep/*.cc
 poly2tri/*.cc
 )

add_library(${PROJECT_NAME}
 SHARED
 ${poly2tri}
 )



set(BOOST_ROOT "C:/Program Files/PCL 1.11.1/3rdParty/Boost")
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



find_package(PCL REQUIRED)
#PCL
include_directories (${PCL_INCLUDE_DIRS})
include_directories (${OpenCV_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})
message("===========${Boost_LIBRARIES}")
message("===========${Boost_INCLUDE_DIRS}")

qt5_use_modules(${PROJECT_NAME} Core Widgets Gui OpenGL )

find_package(flann)
#link_directories(${PCL_LIBRARY_DIRS} "lib")
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS} 
#${Boost_INCLUDE_DIRS}
${FLANN_INCLUDE_DIR}
)
message("===flann")
message(${FLANN_INCLUDE_DIR})

find_package(nlohmann_json CONFIG REQUIRED)
#    message("===${INTERFACE_INCLUDE_DIRECTORIES}")
#    get_target_property(JSON_INC_PATH nlohmann_json::nlohmann_json INTERFACE_INCLUDE_DIRECTORIES)
#    include_directories(${JSON_INC_PATH})
#    message(${JSON_INC_PATH})

add_library(pcdReader SHARED
    pcdReader.cpp
    testbed/jsonfileopt.cpp
)
set_target_properties(pcdReader PROPERTIES LINK_FLAGS /INCLUDE:__tcmalloc)

target_link_libraries(pcdReader
  nlohmann_json::nlohmann_json # head only lib will auto add include path?
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
   ${Boost_LIBRARIES}
   libtcmalloc_minimal
)

add_executable(tri 
 testbed/pcl.cc
 testbed/jsonfileopt.cpp
 )

 find_package(Boost REQUIRED COMPONENTS log system program_options thread)

target_link_libraries(tri
  pcdReader
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
   ${Boost_LIBRARIES}
)