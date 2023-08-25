cmake_minimum_required(VERSION 3.12)

project(MyArmor)

# 设置 OpenCV 的路径
set(OpenCV_DIR "D:/Visual Studio 2022/opencv/build")
include_directories(${OpenCV_INCLUDE_DIR})

# 查找并加载 OpenCV
find_package(OpenCV REQUIRED)

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/source-charset:utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(armor_detector STATIC ${PROJECT_SOURCE_DIR}/src/ArmorDector.cpp)
add_library(armor_classifier STATIC ${PROJECT_SOURCE_DIR}/src/ArmorNumClassifier.cpp)
add_library(find_light STATIC ${PROJECT_SOURCE_DIR}/src/FindLight.cpp)
add_library(pnp_solver STATIC ${PROJECT_SOURCE_DIR}/src/pnp_solver.cpp)
add_library(serial STATIC ${PROJECT_SOURCE_DIR}/src/Serial.cpp)

add_executable(MyArmor ${SRC})
target_link_libraries(MyArmor 
    armor_detector 
    armor_classifier 
    find_light 
    pnp_solver 
    serial 
    ${OpenCV_LIBS}  # 链接 OpenCV 库
)

