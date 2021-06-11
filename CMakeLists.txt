cmake_minimum_required(VERSION 3.10)
project(TBlock)

set(CMAKE_CXX_STANDARD 14)

AUX_SOURCE_DIRECTORY(. SRC)

add_executable(TBlock ${SRC})

add_definitions("-g")
if(!WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=AddressSanitizer")
endif()

