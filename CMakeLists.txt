cmake_minimum_required(VERSION 3.10)
project(MTF01Reader C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(zenohc REQUIRED)

include_directories(include)

add_executable(mtf01_reader
    src/main.c
    src/mtf01.c
    src/config.c
    src/serial.c
)

target_compile_options(mtf01_reader PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(mtf01_reader zenohc::lib)
