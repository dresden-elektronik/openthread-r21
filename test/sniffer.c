int main(int argc, char const *argv[])
{
    samr21Nvm_init();
    samr21ClockTrxSrcInit();
    samr21Trx_interfaceInit();

    samr21Trx_setupMClk(0x5); //MCLK 1MHz -> 16 Mhz
    samr21ClockInitAfterTrxSetup();

    samr21TimerInit();

    samr21DebugPortsInit();
    samr21RadioInit();  

    samr21Usb_init();


    while (/* condition */)
    {
        /* code */
    }
    


}