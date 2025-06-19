# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(presence_rd_03e)

target_sources(app PRIVATE src/main.cpp)
target_link_libraries(app PRIVATE c stdc++ supc++)
target_compile_options(app PRIVATE -fmacro-backtrace-limit=0)
