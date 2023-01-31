# OpenThread-R21


An OpenThread Border Router firmware port for Atmel-R21 (Microchip samr21) based ConBee II and RaspBee II.  
**Note:** the port is still development, watch this repository to get notified when first beta test version is released.

* The firmware converts the **ConBeeII**/**RaspbeeII** into a Radio-Co-Processor (RCP) for a Thread Border-Router-Daemon running on a Host. This functions as a Gateway between Thread-Devices and the local Network.
* Thread forms the underlying technology for 802.15.4 based Matter-Devices 
* The firmware can be flashed via the GCFFlasher in the same way as deCONZ and ZShark firmware is flashed.
* Any deCONZ settings stored in NVRAM are preserved (**in development, no guarantees, backup your data!**).
* The firmware is **Thread-RCP only**, it **can not** be used in parallel with Zigbee firmware (but switching the firmware is always possible).


# Build Guide (RCP-Firmware):

This Part describes how to build the openthread rcp-firmware from source. **This will be obsolete once the .gcf image is available.**


1. Clone this Repo:
```console
git clone https://github.com/dresden-elektronik/openthread-r21
```

2. cd into the directory and init the openthread and tinyusb submodule
```console
cd openthread-r21
git submodule update --init openthread/
git submodule update --init third_party/tinyusb
```

3. execute the bootstrap script (This will install necessary dependencies and download the arm-none-eabi-gcc Toolchain)
```console
bash script/bootstrap.sh
```
> If your are on an unsupported Platform, you will have to manually install the following dependencies: 
> automake, g++, libtool, make, cmake, ninja-build, shellcheck 

4. build the RCP-Firmware
```console
bash script/build.sh
```

5. Done! You will find the linked Firmware (.elf Format) at **/openthread/out/build/ot-rcp**


# Flash Firmware via OpenOCD and GDB

This Part describes how to flash the previously build rcp-firmware to evalBoard by using a OpenOCD compatible Debugging Tool.


0. Install [OpenOCD](https://openocd.org/) (**This Step may be skipped on debian and fedora based Systems**, bootstrap.sh should install OpenOCD via apt or dnf )

1. Find the Open-OCD configuration file for your Debugger (Checkout [eLinux](https://elinux.org/OpenOCD_Config_File_Paths)-Page where to find your config File)
> A Configuration for the ATMEL-ICE and the ATSAMR21ZLL-EK are provided in script/openocd_config/. You can also use them as a Template for your config

2. Start an OpenOCD-Server (**Do this in a seperate Terminal**, this may require sudo-rights)
    **Make Sure Your Devices are Powered!** 
```console
#For Atmel-ICE
(sudo) openocd -f script/openocd_config/ATMEL-ICE-OpenOCD-samr21e18a.cfg

#Generic Debugger
(sudo) openocd -f <yourDebuggerConfigFile>.cfg
```
If everything goes well, you should see the following Output:
```console
Info : Listening on port 3333 for gdb connections
```
3. Start the GDB-Debugger, Connect to the OpenOCD-Server and flash the Firmware-File

```console
gcc-arm-none-eabi/bin/arm-none-eabi-gdb \
    --init-eval-command='target extended-remote localhost:3333' \
    ./out/build/bin/ot-rcp \
    --eval-command='load'
```

4. Reset the MCU (via gdb shell)
```console
(gdb) r
```

5. Exit GDB by pressing **CTRL + C** and typing quit into the GDB Shell:
```console
(gdb) quit
```

6. Done! The Firmware on your Device is ready to act as an Openthread-RCP Dongle


# Install-Guide OpenThread Border-Router Daemon (Program running on the Host Side)

This Part describes how to prepare a host Platform to operate as a Openthread Border Router in conjunction with the connected RCP-Device.
**It is strongly advised to follow the Guide on the Official Openthread Website.**  
## [Official Openthread Border Router Guide](https://openthread.io/guides/border-router)
Tested on Debian11/Ubuntu22.04LTS/RaspberryPiOS(32Bit)

0. Start with a fresh OS-Installation on whatever Platform you are planning to Use (Like RaspberryPi, VM, NUC). 

    The OpenThread Border Router works best with debain based Platforms

1. On your desired Host-Platform clone into the Openthread-Borader-Router-[repository](https://github.com/openthread/ot-br-posix)

```console
git clone https://github.com/openthread/ot-br-posix
```

2. cd into the repository and run the bootstrap script

```console
cd ot-br-posix
(sudo) bash script/bootstrap
```

3. Install the Border Router Daemon and specify the interface used to communicate with the local network
```console
#Raspberry Pi on Ethernet
(sudo) bash INFRA_IF_NAME=eth0 ./script/setup

#Raspberry Pi on Wifi
(sudo) bash INFRA_IF_NAME=wlan0 ./script/setup
```
4. Check if the serial connection got initialized correctly
```console
ls /dev/ | grep tty*
```
should return something like 
```console
ttyACM0
```
or
```console
ttyACM1
```

5. Modify the otbr-agent settings to use the expected serial-device for communication with the RCP
```console
(sudo) nano /etc/default/otbr-agent
```

The Device will likely aper as /dev/ttyACM0 but you should double check that with something like **dmesg**.
You should modify your config-file according to your system. Some examples:

WIFI and rcp mapped to /dev/ttyACM0
```console
OTBR_AGENT_OPTS="-I wpan0 -B eth0 spinel+hdlc+uart:///dev/ttyACM0 trel://wlan0"
```

Ethernet and RCP mapped to /dev/ttyACM1
```console
OTBR_AGENT_OPTS="-I wpan0 -B eth0 spinel+hdlc+uart:///dev/ttyACM1 trel://eth0"
```

6. After modifying the otbr-agent config-file you should reboot your Host.
```console
sudo reboot now
```
    
7. After Reboot you should be able to reach the Openthread Border Router Web-Interface by typing 
    `http://YOURHOSTADDR/`
    into your Web-Browser

