cmake_minimum_required(VERSION 3.10)
project(TBlock)

set(CMAKE_CXX_STANDARD 14)

AUX_SOURCE_DIRECTORY(. SRC)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

#find_package(GEOS REQUIRED)

FIND_PATH(GEOS_INCLUDE_DIR geos_c.h "$ENV{LIB_DIR}/include" /usr/local/include /usr/include C:/GEOS/include)
FIND_LIBRARY(GEOS_LIBRARY NAMES geos_c PATHS "$ENV{LIB_DIR}/lib" /usr/local/lib /usr/lib C:/GEOS/lib)


include_directories(${GEOS_INCLUDE_DIR})

add_executable(TBlock ${SRC})

target_link_libraries(TBlock ${GEOS_LIBRARY})

add_definitions("-g")
if(!WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=AddressSanitizer")
endif()