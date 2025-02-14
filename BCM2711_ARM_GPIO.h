#define _FILE_OFFSET_BITS  64

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
// #define PWM0_REG_BASE_ADDRESS (0x7e20c000)
#define PWM0_REG_BASE_ADDRESS (0x7E200000)
// #define PWM0_REG_BASE_ADDRESS (0xFE000000)
// VPU: 0x7E00B880  (same for AArch32)
// ARM64: 0xFE00B880
// #define PWM1_REG_BASE_ADDRESS (0x7e20c800)
// #define PWM1_REG_BASE_ADDRESS (0xFE000800)
#define PWM1_OFFSET (2048) // 0x800 = 2048

#define REGISTER_SIZE_IN_BYTES (4)

// Offsets for PWM registers
#define PWM_CTL_REG_OFFSET  (0x00) // Control
#define PWM_STA_REG_OFFSET  (0x04) // Status (useful for debugging)
#define PWM_DMAC_REG_OFFSET (0x08) // DMA Configuration
#define PWM_RNG1_REG_OFFSET (0x10) // Channel 1 range
#define PWM_DAT1_REG_OFFSET (0x14) // Channel 1 data
#define PWM_FIF1_REG_OFFSET (0x18) // FIFO input
#define PWM_RNG2_REG_OFFSET (0x20) // Channel 2 range
#define PWM_DAT2_REG_OFFSET (0x24) // Channel 2 data

// Define PWM0 (Channel 0) Offsets
#define PWM0_CTL_REG_INDEX  (PWM_CTL_REG_OFFSET / REGISTER_SIZE_IN_BYTES)
#define PWM0_STA_REG_INDEX  (PWM_STA_REG_OFFSET / REGISTER_SIZE_IN_BYTES) 
#define PWM0_DMAC_REG_INDEX (PWM_DMAC_REG_OFFSET / REGISTER_SIZE_IN_BYTES)
#define PWM0_RNG1_REG_INDEX (PWM_RNG1_REG_OFFSET / REGISTER_SIZE_IN_BYTES) // defines channel 1 range (leave as default 32)
#define PWM0_DAT1_REG_INDEX (PWM_DAT1_REG_OFFSET / REGISTER_SIZE_IN_BYTES) // We will use this register to set duty cycle for PWM channel 1 (PWM modulator algorithm outlined in spec)
#define PWM0_FIF1_REG_INDEX (PWM_FIF1_REG_OFFSET / REGISTER_SIZE_IN_BYTES) // Not using the FIFO for our implementation
#define PWM0_RNG2_REG_INDEX (PWM_RNG2_REG_OFFSET / REGISTER_SIZE_IN_BYTES) // defines channel 2 range (leave as default 32)
#define PWM0_DAT2_REG_INDEX (PWM_DAT2_REG_OFFSET / REGISTER_SIZE_IN_BYTES) // We will use this register to set duty cycle for PWM channel 2 (PWM modulator algorithm outlined in spec)

// Define PWM1 (Channel 1) Offsets
#define PWM1_CTL_REG_INDEX (PWM1_OFFSET + PWM0_CTL_REG_INDEX)
#define PWM1_STA_REG_INDEX (PWM1_OFFSET + PWM0_STA_REG_INDEX)
#define PWM1_DMAC_REG_INDEX (PWM1_OFFSET + PWM0_DMAC_REG_INDEX)
#define PWM1_RNG1_REG_INDEX (PWM1_OFFSET + PWM0_RNG1_REG_INDEX) // defines channel 1 range (leave as default 32)
#define PWM1_DAT1_REG_INDEX (PWM1_OFFSET + PWM0_DAT1_REG_INDEX) // We will use this register to set duty cycle for PWM channel 1 (PWM modulator algorithm outlined in spec)
#define PWM1_FIF1_REG_INDEX (PWM1_OFFSET + PWM0_FIF1_REG_INDEX) // Not using the FIFO for our implementation
#define PWM1_RNG2_REG_INDEX (PWM1_OFFSET + PWM0_RNG2_REG_INDEX) // defines channel 2 range (leave as default 32)
#define PWM1_DAT2_REG_INDEX (PWM1_OFFSET + PWM0_DAT2_REG_INDEX) // We will use this register to set duty cycle for PWM channel 2 (PWM modulator algorithm outlined in spec)

// bit 31:16 = reserved
// bit 15 = MSEN2 (Channel 2) M/S enable - 0: PWM algorithm is used, 1 M/S transmission is used
// bit 14 = reserved
// bit 13 = USEF2 (Channel 2) Use FIFO? - 0: Data register is used, 1 FIFO is used
// bit 12 = POLA2 (Channel 2) Polarity - 0: 0 = low, 1 = high
// bit 11 = SBIT2 (Channel 2) Silence Bit: Defines state of output when no transmission takes place (default 0)
// bit 10 = RPTL2 (Channel 2) repeat last data 0 when FIFO is empty, 1 last data in FIFO is transmitted repeated until FIFL isn't empty
// bit 9  = MODE2 (Channel 2) 0: PWM mode
// bit 8  = PWEN2 (Channel 2) 0: Channel disabled, 1: Channel enabled
// bit 7  = MSEN1
// bit 6  = CLRF  (1: clear fifo, 0: no effect)
// bit 5  = USEF1
// bit 4  = POLA1
// bit 3  = SBIT1
// bit 2  = RPTL1
// bit 1  = MODE1
// bit 0  = PWEN1
// Cureent value = 0000000100000001 = 0x101
#define CTL_REG_VALUE (0x101) // See bit definitions above

// 100% duty cycle -> all bits in the register = 1 (32 bits -> 0b11111111111111111111111111111111)
#define DUTY_CYCLE_100P (0xFFFFFFFF) // 0b11111111111111111111111111111111
#define DUTY_CYCLE_96P  (0xFFFF7FFF)
#define DUTY_CYCLE_50P  (0x55555555) // 0b01010101010101010101010101010101
