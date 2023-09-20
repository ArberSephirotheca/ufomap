cmake_minimum_required(VERSION 3.9)

project(ufomap_benchmark LANGUAGES CXX)
#LINK_DIRECTORIES(/home/zheyuan/ufomap/ufomap/build)
#list(APPEND CMAKE_MODULE_PATH "/home/zheyuan/ufomap/ufomap/cmake")
#set(UFO_LIBRARY "/home/zheyuan/ufomap/ufomap/build")
#find_library(UFO_LIBRARY ufo_library)
include_directories("/home/zheyuan/ufomap/ufomap/include")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--copy-dt-needed-entries")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(TBB REQUIRED)
add_executable(my_app)

target_sources(
  my_app
  PRIVATE main.cpp)
target_link_libraries(my_app /home/zheyuan/ufomap/ufomap/build/libMap.so)


target_compile_features(my_app PUBLIC cxx_std_17)
target_compile_options(my_app PRIVATE -Wall -ltbb)
