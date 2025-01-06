#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h> // Need for fd flags
#include <sys/mman.h> // mmap() defined in this header
#include <unistd.h> // Need to close fd and for sleep function

#include "BCM2711_ARM_GPIO.h"

#define SUCCESS (0)

// These help with the bit shifting and arithmetic to properly modify the correct GPIO later in the program
#define FSEL_NUM_BITS (3)
#define GPIO_PINS_PER_REG (10)

// These are the pins corresponding to the left side drive train
#define BL_MOTOR_PIN_A (17)
#define BL_MOTOR_PIN_B (27)
#define FL_MOTOR_PIN_A (22)
#define FL_MOTOR_PIN_B (23)

// Define the Register Index which will be used to set the motor pins to output
#define BL_RI_PIN_A (BL_MOTOR_PIN_A / GPIO_PINS_PER_REG)
#define BL_RI_PIN_B (BL_MOTOR_PIN_B / GPIO_PINS_PER_REG)
#define FL_RI_PIN_A (FL_MOTOR_PIN_A / GPIO_PINS_PER_REG)
#define FL_RI_PIN_B (FL_MOTOR_PIN_B / GPIO_PINS_PER_REG)

// This Register will be used to set the pin high
#define GPIO_SET_REG (OFFSET_GPFSEL0 / GPIO_REG_SIZE)

// This Register will be used to set the pin low
#define GPIO_CLR_REG (OFFSET_CLR0 / GPIO_REG_SIZE)

int handle_error(char * msg) {
    perror(msg); 
    return EXIT_FAILURE;
}

int get_gpio_virtual_mem_block(volatile uint32_t * gpios) {
    // In the filesystem we have /dev/gpiomem which is the block of physical memory we need to map
    // Man page: https://man7.org/linux/man-pages/man2/open.2.html 
    int gpiomem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (gpiomem_fd < 0) {
        return handle_error("open() /dev/gpiomem failed");
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
        return handle_error("mmap() failed");
    }

    // Type cast gpio mem to volatile since GPIO state is not guarunteed
    gpios = (volatile uint32_t *)gpio_reg_map;
    return gpiomem_fd;
}

int init_gpio_pins(volatile uint32_t * gpios) {
    // Setup BL motor pin A
    uint32_t fsel_shift = (BL_MOTOR_PIN_A % GPIO_PINS_PER_REG) * FSEL_NUM_BITS;
    gpios[BL_RI_PIN_A] &= (0b000 << fsel_shift); // Clear current function by forcing bits to 000
    gpios[BL_RI_PIN_A] |= (0b001 << fsel_shift); // Change GPIO to an output

    // Setup BL motor pin B
    fsel_shift = (BL_MOTOR_PIN_B % GPIO_PINS_PER_REG) * FSEL_NUM_BITS;
    gpios[BL_RI_PIN_B] &= (0b000 << fsel_shift); // Clear current function by forcing bits to 000
    gpios[BL_RI_PIN_B] |= (0b001 << fsel_shift); // Change GPIO to an output
}

void moveLeftSideForward(volatile uint32_t * gpios) {
    printf("\nMoving Left Side Forward...");
    gpios[GPIO_SET_REG] = (1 << BL_MOTOR_PIN_A); // Back left motor
    gpios[GPIO_CLR_REG] = (1 << BL_MOTOR_PIN_B);

    gpios[GPIO_SET_REG] = (1 << FL_MOTOR_PIN_A); // Front left motor
    gpios[GPIO_CLR_REG] = (1 << FL_MOTOR_PIN_B);
}

void stop(volatile uint32_t * gpios) {
    printf("\nStopping Left Side Drive Train...");
    gpios[GPIO_CLR_REG] = (1 << BL_MOTOR_PIN_A); // Back left motor
    gpios[GPIO_CLR_REG] = (1 << BL_MOTOR_PIN_B);

    gpios[GPIO_CLR_REG] = (1 << FL_MOTOR_PIN_A); // Front left motor
    gpios[GPIO_CLR_REG] = (1 << FL_MOTOR_PIN_B);
}

void moveLeftSideBackward(volatile uint32_t * gpios) {
    printf("\nMoving Left Side Backward...");
    gpios[GPIO_SET_REG] = (1 << BL_MOTOR_PIN_A); // Back left motor
    gpios[GPIO_CLR_REG] = (1 << BL_MOTOR_PIN_B);

    gpios[GPIO_SET_REG] = (1 << FL_MOTOR_PIN_A); // Front left motor
    gpios[GPIO_CLR_REG] = (1 << FL_MOTOR_PIN_B);
}

int main() 
{
    volatile uint32_t gpios;
    int gpiomem_fd = get_gpio_virtual_mem_block(&gpios);
    init_gpio_pins(&gpios);

    // Test...
    uint32_t count = 0;
    do {
        moveForward(&gpios);
        sleep(1);
        stop(&gpios);
        sleep(1);
        moveBackward(&gpios);
        sleep(1);
        count++;
    } while (count < 3);

    munmap(&gpios, GPIO_PUP_PDN_CNTRL_REG3);
    close(gpiomem_fd);
    printf("\n");

    return SUCCESS;
}