#!/bin/bash
export USE_CUSTOM_LIBCXX=1
export USE_NRF52=1
source /home/theorlangur/ncs/toolchains/7cbc0036f4/env_llvm_m4_lcxx.sh
$*
