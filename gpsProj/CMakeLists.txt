cmake_minimum_required(VERSION 3.12)
project(gpsdReader)

set(CMAKE_CXX_STANDARD 14)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(gpsdReader main.cpp)
target_link_libraries(gpsdReader Threads::Threads)