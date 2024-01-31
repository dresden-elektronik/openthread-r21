#!/bin/bash

scriptDir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
set -euxo pipefail

OT_CMAKE_NINJA_TARGET=${OT_CMAKE_NINJA_TARGET:-}

OT_SRCDIR="${scriptDir}/.."
readonly OT_SRCDIR

OT_OPTIONS=(
    "-DCMAKE_TOOLCHAIN_FILE=${OT_SRCDIR}/src/arm-none-eabi-gcc.cmake"
    "-DCMAKE_BUILD_TYPE=Release"
    "-DOT_PLATFORM=external"
    "-DOT_SLAAC=ON"
    "-DOT_APP_RCP=ON"
    "-DOT_RCP=ON"
    "-DOT_FTD=OFF"
    "-DOT_MTD=OFF"
    "-DOT_APP_CLI=OFF"
    "-DOT_APP_NCP=OFF"
    "-DDDEL_GCF_BUILD=ON"
    "-DTARGET_DEVICE=RASPBEE2"
)
readonly OT_OPTIONS

build()
{
    local builddir="${OT_CMAKE_BUILD_DIR:-${OT_SRCDIR}/out/rcpRaspbee2}"


    mkdir -p "${builddir}"
    cd "${builddir}"

    cmake -GNinja -DOT_COMPILE_WARNING_AS_ERROR=ON "$@" "${OT_SRCDIR}"

    if [[ -n ${OT_CMAKE_NINJA_TARGET[*]} ]]; then
        ninja "${OT_CMAKE_NINJA_TARGET[@]}"
    else
        ninja
    fi

    cd "${OT_SRCDIR}"
}

main()
{
    local options=("${OT_OPTIONS[@]}")

    options+=("$@")

    build "${options[@]}"

    exec ${OT_SRCDIR}/gcc-arm-none-eabi/bin/arm-none-eabi-objcopy -O binary ${OT_SRCDIR}/out/rcpRaspbee2/bin/ot-rcp ${OT_SRCDIR}/out/rcpRaspbee2/ot-rcp-rb2-gcf.bin
}

main "$@"
