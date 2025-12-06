#!/bin/bash
west build --build-dir build52 . --pristine \
    --board promicro_nrf52840/nrf52840/uf2 -- \
    -DBOARD_ROOT=~/myapps/cpp/nrf \
    -DCONF_FILE="prj.conf config/cpp_lcxx.conf config/nrf52.conf config/zb.conf config/no_log.conf" \
    -DZEPHYR_TOOLCHAIN_VARIANT=llvm \
    -DCONFIG_LLVM_USE_LLD=y \
    -DCONFIG_COMPILER_RT_RTLIB=y \
    -DCMAKE_TOOLCHAIN_FILE=${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
