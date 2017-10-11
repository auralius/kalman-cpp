cmake_minimum_required(VERSION 2.6)

project(kalman-cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ../bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../bin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE Debug)

add_library (kalman-cpp ./kalman/kf.cpp ./kalman/ekf.cpp ./kalman/ekf2.cpp ./kalman/fun.cpp)

target_include_directories (kalman-cpp PUBLIC ./kalman)

add_executable(demo1 ./samples/main1.cpp)
add_executable(demo2 ./samples/main2.cpp)
add_executable(demo3 ./samples/main3.cpp)
add_executable(demo4 ./samples/main4.cpp)
add_executable(demo5 ./samples/main5.cpp)
add_executable(demo6 ./samples/main6.cpp)
add_executable(demo7 ./samples/main7.cpp)
add_executable(demo8 ./samples/main8.cpp)
add_executable(demo9 ./samples/main9.cpp)

install(TARGETS demo1 RUNTIME DESTINATION ../bin)
install(TARGETS demo2 RUNTIME DESTINATION ../bin)
install(TARGETS demo3 RUNTIME DESTINATION ../bin)
install(TARGETS demo4 RUNTIME DESTINATION ../bin)
install(TARGETS demo5 RUNTIME DESTINATION ../bin)
install(TARGETS demo6 RUNTIME DESTINATION ../bin)
install(TARGETS demo7 RUNTIME DESTINATION ../bin)
install(TARGETS demo8 RUNTIME DESTINATION ../bin)
install(TARGETS demo9 RUNTIME DESTINATION ../bin)

target_link_libraries (demo1 LINK_PUBLIC kalman-cpp armadillo)	
target_link_libraries (demo2 LINK_PUBLIC kalman-cpp armadillo)   
target_link_libraries (demo3 LINK_PUBLIC kalman-cpp armadillo)   
target_link_libraries (demo4 LINK_PUBLIC kalman-cpp armadillo)   
target_link_libraries (demo5 LINK_PUBLIC kalman-cpp armadillo)   
target_link_libraries (demo6 LINK_PUBLIC kalman-cpp armadillo)  
target_link_libraries (demo7 LINK_PUBLIC kalman-cpp armadillo python2.7)  
target_link_libraries (demo8 LINK_PUBLIC kalman-cpp armadillo)  
target_link_libraries (demo9 LINK_PUBLIC kalman-cpp armadillo)  