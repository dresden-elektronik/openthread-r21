# OpenThread-R21

An OpenThread Border Router firmware port for R21 based ConBee II and RaspBee II.  
**Note:** the port is still development, watch this repository to get notified when first beta test version is released.

* The firmware converts the ConBee2/Raspbee2 into a Radio-Co-Processor (RCP) for a Thread Border-Router-Deamon running on a Host. This functions as a Gateway between Thread-Devices and the local Network.
* Thread forms the underlying technology for 802.15.4 based Matter-Devices 
* The firmware can be flashed via the GCFFlasher in the same way as deCONZ and ZShark firmware is flashed.
* Any deCONZ settings stored in NVRAM are preserved (**in development, no guarantees, backup your data!**).
* The firmware is Thread only, it **can not** be used in parallel with Zigbee firmware (but switching the firmware is always possible).

