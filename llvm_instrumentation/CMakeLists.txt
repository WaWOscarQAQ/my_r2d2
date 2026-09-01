cmake_minimum_required(VERSION 3.20)
project(r2d2_llvm_instrumentation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(LLVM REQUIRED CONFIG)
find_package(Threads REQUIRED)

if(NOT LLVM_VERSION_MAJOR EQUAL 18)
  message(FATAL_ERROR "R2D2 instrumentation requires LLVM 18, found ${LLVM_PACKAGE_VERSION}")
endif()

list(APPEND CMAKE_MODULE_PATH "${LLVM_CMAKE_DIR}")
include(AddLLVM)

add_llvm_pass_plugin(R2D2Instrumentation
  pass/R2D2Instrumentation.cpp
)

target_include_directories(R2D2Instrumentation PRIVATE ${LLVM_INCLUDE_DIRS})
target_compile_definitions(R2D2Instrumentation PRIVATE ${LLVM_DEFINITIONS})

add_library(r2d2_llvm_runtime SHARED
  runtime/R2D2Runtime.cpp
  ../tracer/src/tracers.cpp
)
target_include_directories(r2d2_llvm_runtime PRIVATE include ../tracer/include)
target_link_libraries(r2d2_llvm_runtime PRIVATE Threads::Threads)
target_compile_options(r2d2_llvm_runtime PRIVATE -Wall -Wextra -Wpedantic)
