# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(EXTRA_DTC_OVERLAY_FILE "dts.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(presence_dfr_c4001)

target_include_directories(app PRIVATE submodules/esp_generic_lib/include)

target_sources(app PRIVATE src/main.cpp)
add_subdirectory(src/lib)

if(DEFINED ENV{USELIBCPP})
    target_link_libraries(app PRIVATE c c++ c++abi)
else()
    target_link_libraries(app PRIVATE c stdc++ supc++)
    target_compile_options(app PRIVATE -fmacro-backtrace-limit=0)
    target_compile_options(app PRIVATE -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof)
endif()

