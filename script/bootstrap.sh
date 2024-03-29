#!/bin/bash
scriptDir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ARCH=$(uname -m)

if [ ! -d "${scriptDir}/../gcc-arm-none-eabi" ]; then
    # Take action if $DIR exists. #
    echo "No Toolchain found!"
    echo "Downloading Arm M0+ Toolchain to:"
    echo "${scriptDir}/../gcc-arm-none-eabi"
    wget "https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-${ARCH}-linux.tar.bz2" \
        -O ${scriptDir}/../gcc-arm-none-eabi.tar.bz2
    tar xfvj ${scriptDir}/../gcc-arm-none-eabi.tar.bz2
    rm ${scriptDir}/../gcc-arm-none-eabi.tar.bz2
    mv gcc-arm-none-eabi-10.3-2021.10 gcc-arm-none-eabi 
fi


if command -v apt-get; then
    sudo apt install openocd
elif command -v rpm; then
    sudo dnf install openocd
elif command -v pacman; then
    sudo pacman -S openocd
else
    echo No Known Package Manager found, Please install OpenOCD manualy
    
fi

bash "${scriptDir}/../openthread/script/bootstrap"
	
