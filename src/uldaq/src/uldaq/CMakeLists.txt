cmake_minimum_required(VERSION 3.0)
project(libuldaq)
add_compile_options(-std=c++11 -g -fPIC -w)

file(GLOB SRC_FILES src/*.cpp  src/*/*.cpp  src/*/*/*.cpp 
                    src/*.h     src/*/*.h    src/*/*/*.h
                    src/*.c     src/*/*.c    src/*/*/*.c
  )

add_library(libuldaq ${SRC_FILES})

target_link_libraries(libuldaq
  usb-1.0
  )

