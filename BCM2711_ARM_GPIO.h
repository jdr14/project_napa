
/**************************
*
* Note this header is written for the Raspberry Pi 4 which uses the BCM2711 SOC
* 
* GPIO - See section 5.2 of the BCM2711 data sheet at 
* https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf
* 
**************************/
#define GPIO_REG_BASE_ADDRESS (0x7e200000)  // GPIO base address
#define GPIO_REG_SIZE (4) // The GPIO has the following 4 byte (32 bit) registers

// Use these Registers to set the function of the GPIO (see datasheet for the specific functionality of each pin)
#define OFFSET_GPFSEL0 (0x00)  // FSelect 0: GPIO 9 - 0
#define OFFSET_GPFSEL1 (0x04)  // FSelect 1: GPIO 19 - 10
#define OFFSET_GPFSEL2 (0x08)  // FSelect 2: GPIO 29 - 20
#define OFFSET_GPFSEL3 (0x0c)  // FSelect 3: GPIO 39 - 30
#define OFFSET_GPFSEL4 (0x10)  // FSelect 4: GPIO 49 - 40
#define OFFSET_GPFSEL5 (0x14)  // FSelect 5: GPIO 57 - 50 (57 starts at bit 23:21)

// Use these registers to set a GPIO
#define OFFSET_GPSET0  (0x1c)  //  0 - 31
#define OFFSET_GPSET1  (0x20)  // 32 - 57

// Registers to clear GPIOs: Setting the bit to 1 clears the GPIO
#define OFFSET_CLR0 (0x28)  //  0 - 31
#define OFFSET_CLR1 (0x2c)  // 32 - 57
// etc...
// TODO: Fill out the rest of this GPIO header according to the datasheet

// This is also the last address in the GPIO memory block, so this can double as the block size
#define GPIO_PUP_PDN_CNTRL_REG3 (0xf0) // GPIO Pull-up/Pull-down Register 3


/**************************
*
* PWM - See Chapter 8
* 
**************************/
// Each of the following addresses are the base address for the registers related to the channels of the PWM
#define PWM0_REG_BASE_ADDRESS (0x7e20c000)
#define PWM1_REG_BASE_ADDRESS (0x7e20c800)

// Offsets for PWM registers
#define PWM_CTL_REG_OFFSET  (0x00) // Control
#define PWM_STA_REG_OFFSET  (0x04) // Status
#define PWM_DMAC_REG_OFFSET (0x08) // DMA Configuration
#define PWM_RNG1_REG_OFFSET (0x10) // Channel 1 range
#define PWM_DAT1_REG_OFFSET (0x14) // Channel 1 data
#define PWM_FIF1_REG_OFFSET (0x18) // FIFO input
#define PWM_RNG2_REG_OFFSET (0x20) // Channel 2 range
#define PWM_DAT2_REG_OFFSET (0x24) // Channel 2 data
