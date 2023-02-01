int main(int argc, char const *argv[])
{
    samr21NvmInit();
    samr21ClockTrxSrcInit();
    samr21TrxInterfaceInit();

    samr21TrxSetupMClk(0x5); //MCLK 1MHz -> 16 Mhz
    samr21ClockInitAfterTrxSetup();

    samr21TimerInit();

    samr21DebugPortsInit();
    samr21RadioInit();  

    samr21UsbInit();


    while (/* condition */)
    {
        /* code */
    }
    


}