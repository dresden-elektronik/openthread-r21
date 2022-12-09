flashTestRX:
	cd ./test && make flashRX

flashTestTX:
	cd ./test && make flashTX

flashTestED:
	cd ./test && make flashED
	
flashTestMisc:
	cd ./test && make flashMisc

testGCF:
	cd ./test && make testGCF
	./gcc-arm-none-eabi/bin/arm-none-eabi-objcopy -O binary ./out/images/testGCF.elf ./out/images/testGCF.bin

getToolchain:
	wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
	tar xfvj gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 
	mv gcc-arm-none-eabi-10.3-2021.10 gcc-arm-none-eabi
	rm gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
	

cleanAll:
	rm ./out/* -rf

killOCD:
	cd ./test && make killOCD