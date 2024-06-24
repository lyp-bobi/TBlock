cmake_minimum_required(VERSION 3.10)
project(TBlock)

set(CMAKE_CXX_STANDARD 14)

set (CMAKE_CXX_FLAGS "$ENV{CMAKE_CXX_FLAGS} -std=c++14 -O3")
set (CMAKE_CXX_FLAGS_DEBUG  "$ENV{CMAKE_CXX_FLAGS}  -g -O0")

#set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

#find_package(GEOS REQUIRED)

#FIND_PATH(GEOS_INCLUDE_DIR geos_c.h "$ENV{LIB_DIR}/include" /usr/local/include /usr/include C:/GEOS/include)
#FIND_LIBRARY(GEOS_LIBRARY NAMES geos_c PATHS "$ENV{LIB_DIR}/lib" /usr/local/lib /usr/lib C:/GEOS/lib)

include_directories(.)

#add_subdirectory(sqlite)
#include_directories(sqlite)
#include_directories(${GEOS_INCLUDE_DIR})

aux_source_directory(. SRC)
add_subdirectory(test)

#add_subdirectory(fpzip)



#target_link_libraries(TBlock fpzip)



#target_link_libraries(TBlock ${GEOS_LIBRARY})


add_library(TBlock SHARED ${SRC})

install(
        TARGETS TBlock
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)


#add_definitions("-g")
#if(!WIN32)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=AddressSanitizer")
#endif()