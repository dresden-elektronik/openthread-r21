// Author ERH for dresden elektronik ingenieurtechnik gmbh © 2022
#include "compiler.h"

#define PORT_BASE_ADDR        0x41004400


#define PORT_DIR_OFFSET       0x00
#define PORT_DIRCLR_OFFSET    0x04
#define PORT_DIRSET_OFFSET    0x08
#define PORT_DIRTGL_OFFSET    0x0C

#define PORT_OUT_OFFSET       0x10
#define PORT_OUTCLR_OFFSET    0x14
#define PORT_OUTSET_OFFSET    0x18
#define PORT_OUTTGL_OFFSET    0x1C

#define PORT_IN_OFFSET        0x20

#define PORT_CTRL_OFFSET      0x24
#define PORT_WRCONFIG_OFFSET  0x28

#define PORT_PMUXx_OFFSET(x)  (0x30 + x)

#define PORT_PINCFGx_OFFSET(x)  (0x40 + x)

typedef enum {
    PORT_BANK_A = 0x00,
    PORT_BANK_B = 0x01,
    PORT_BANK_C = 0x02
}PortGroup;

typedef enum {
    PORT_PIN_MUX_A     = 0x00,
    PORT_PIN_MUX_B     = 0x01,
    PORT_PIN_MUX_C     = 0x02,
    PORT_PIN_MUX_D     = 0x03,
    PORT_PIN_MUX_E     = 0x04,
    PORT_PIN_MUX_F     = 0x05,
    PORT_PIN_MUX_G     = 0x06,
    PORT_PIN_MUX_H     = 0x07,
    PORT_PIN_MUX_NONE  = 0xFF
}PortMuxConfig;

typedef enum {
    PORT_PIN_DISABLED          = 0x00,
    PORT_PIN_PULLDOWN          = 0x01,
    PORT_PIN_PULLUP            = 0x02,
    PORT_PIN_INPUT             = 0x03,
    PORT_PIN_INPUT_PULLDOWN    = 0x04,
    PORT_PIN_INPUT_PULLUP      = 0x05,  
    PORT_PIN_OUTPUT            = 0x06, 
    PORT_PIN_OUTPUT_INPUT      = 0x07,
    PORT_PIN_OUTPUT_VIA_PULL   = 0x08
}PortSetupConfig;


bool setupPortPin(uint8_t PinNum, PortBank PinBank, PortSetupConfig PinSetupCofig, PortMuxConfig PinMuxConfig);
void changeOutputLevel(uint8_t PinNum, PortBank PinBank, bool newOutputLevel);
bool readInputLevel(uint8_t PinNum, PortBank PinBank);

