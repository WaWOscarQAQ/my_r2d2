cmake_minimum_required(VERSION 3.16)
project(tracer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_library(tracer STATIC src/tracers.cpp)
target_include_directories(tracer PUBLIC include)
target_link_libraries(tracer PUBLIC Threads::Threads)
target_compile_options(tracer PRIVATE -Wall -Wextra)

add_executable(mock_writer src/mock_writer.cpp)
target_link_libraries(mock_writer PRIVATE tracer)
