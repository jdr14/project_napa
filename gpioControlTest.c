

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h> // Need for fd flags
#include <sys/mman.h> // mmap() defined in this header
#include <unistd.h> // Need to close fd and for sleep function
// #include <wiringPi.h>

/**************************
*
* Note this code is written for the Raspberry Pi which uses the BCM2711 SOC
* 
* See section 5.2 of the BCM2711 data sheet at 
* https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf
* 
**************************/
#define GPIO_REG_BASE_ADDRESS (0x7e200000)  // GPIO base address
#define GPIO_REG_SIZE (4) // The GPIO has the following 4 byte (32 bit) registers

#define OFFSET_GPFSEL0 (0x00)  // GPIO Function Select 0 (gpio 9 -> 0)
#define OFFSET_GPFSEL1 (0x04)  // GPIO Function Select 1 (gpio 19 -> 10)
#define OFFSET_GPFSEL2 (0x08)  // GPIO Function Select 2 (gpio 29 -> 20)
#define OFFSET_GPFSEL3 (0x0c)  // GPIO Function Select 3
#define OFFSET_GPFSEL4 (0x10)  // GPIO Function Select 4
#define OFFSET_GPFSEL5 (0x14)  // GPIO Function Select 5

// These help with the bit shifting and arithmetic to properly modify the correct GPIO later in the program
#define FSEL_NUM_BITS (3)
#define GPIO_PINS_PER_REG (10)

#define OFFSET_GPSET0 (0x1c)  // GPIO Pin Output Set 0
#define OFFSET_GPSET1 (0x20)  // GPIO Pin Output Set 1 

#define OFFSET_CLR0 (0x28)  // Register to clear GPIOs
#define OFFSET_CLR1 (0x2c)  // Register to clear GPIOs
// etc...

#define GPIO_PUP_PDN_CNTRL_REG3 (0xf0) // GPIO Pull-up/Pull-down Register 3

int handle_error(char * msg) {
    perror(msg); 
    return EXIT_FAILURE;
}

int main()
{
    // In the filesystem we have /dev/gpiomem which is the block of physical memory we need to map
    // Man page: https://man7.org/linux/man-pages/man2/open.2.html 
    int gpiomem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);

    if (gpiomem_fd < 0) {
        handle_error("open() /dev/gpiomem failed");
    }

    // mmap() syscall creates a new mapping in the virtual address space of the calling process
    // Man page: https://man7.org/linux/man-pages/man2/mmap.2.html
    void * gpio_reg_map = (void *)mmap(
        NULL,                    // Allow kernel to decide the virtual base address
        GPIO_PUP_PDN_CNTRL_REG3, // This is the last register in the GPIO memory block
        PROT_READ | PROT_WRITE,  // Enable read and write access
        MAP_SHARED,              // Updates to mapping are visible by other processes, which is what we want with GPIO
        gpiomem_fd,              // FD we created above
        GPIO_REG_BASE_ADDRESS);  // The start address of the GPIO memory block

    if (gpio_reg_map == MAP_FAILED) {
        close(gpiomem_fd);
        handle_error("mmap() failed");
    }

    // At this point, we have successfully mapped gpio registers to an array and have read/write access
    // to manipulate them according to our application needs
    volatile uint32_t * gpios = (volatile uint32_t *)gpio_reg_map;

    uint32_t gpio_pin = 17;
    uint32_t fsel_shift = (gpio_pin % GPIO_PINS_PER_REG) * FSEL_NUM_BITS;

    printf("\nUsing bitshift of %i", fsel_shift);

    // Clear current function by forcing bits to 000
    gpios[1] &= (0b000 << fsel_shift);

    // Change GPIO to an output
    gpios[1] |= (0b001 << fsel_shift);

    uint8_t blink_count = 0; 
    uint32_t set_reg_index = (OFFSET_GPSET0 / GPIO_REG_SIZE);
    uint32_t clr_reg_index = (OFFSET_CLR0 / GPIO_REG_SIZE);
    do {
        printf("\nSetting GPIO %i low with \'off\' register index %i", gpio_pin, clr_reg_index);
        gpios[clr_reg_index] = (1 << gpio_pin); // Set GPIO bit to high in the clr register to clear
        sleep(1); // Sleep 1s

        printf("\nSetting GPIO %i high with \'on\' register index %i", gpio_pin, set_reg_index);
        gpios[set_reg_index] = (1 << gpio_pin); //(0b1 << gpio_pin); // Set GPIO bit in register to high
        sleep(1); // Sleep 1s

        blink_count++;
    } while(blink_count < 5); // only blink 5 times, then exit

    // Don't forget to unmap and close the gpiomem file descriptor
    munmap(gpio_reg_map, GPIO_PUP_PDN_CNTRL_REG3); // Use the offset for the last GPIO register in the block as the size
    close(gpiomem_fd);
    printf("\n");

    return 0;
}