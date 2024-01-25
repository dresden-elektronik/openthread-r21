openocd -f ./test/OpenOCD_config/ATMEL-ICE-OpenOCD-samr21e18a.cfg &
./gcc-arm-none-eabi/bin/arm-none-eabi-gdb --init-eval-command='target extended-remote localhost:3333' ./out/rcpRelease/bin/ot-rcp -eval-command='load'
killall openocd
