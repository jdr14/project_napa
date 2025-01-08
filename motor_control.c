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
#define BL_MOTOR_PIN_B (22)
#define FL_MOTOR_PIN_A (23)
#define FL_MOTOR_PIN_B (24)

// Define the Register Index which will be used to set the motor pins to output
#define BL_RI_PIN_A (BL_MOTOR_PIN_A / GPIO_PINS_PER_REG)
#define BL_RI_PIN_B (BL_MOTOR_PIN_B / GPIO_PINS_PER_REG)
#define FL_RI_PIN_A (FL_MOTOR_PIN_A / GPIO_PINS_PER_REG)
#define FL_RI_PIN_B (FL_MOTOR_PIN_B / GPIO_PINS_PER_REG)

// Define the register set shifts
#define BL_SHIFT_A ((BL_MOTOR_PIN_A % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define BL_SHIFT_B ((BL_MOTOR_PIN_B % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define FL_SHIFT_A ((FL_MOTOR_PIN_A % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define FL_SHIFT_B ((FL_MOTOR_PIN_B % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)

// This Register will be used to set the pin high
#define GPIO_SET_REG (OFFSET_GPSET0 / GPIO_REG_SIZE)

// This Register will be used to set the pin low
#define GPIO_CLR_REG (OFFSET_CLR0 / GPIO_REG_SIZE)

int handle_error(char * msg) {
    perror(msg); 
    return EXIT_FAILURE;
}

// int get_gpio_virtual_mem_block(void * gpio_reg_map) {


//     // Type cast gpio mem to volatile since GPIO state is not guarunteed
//     // gpios = (volatile uint32_t *)gpio_reg_map;
//     return gpiomem_fd;
// }

// int init_gpio_pins(volatile uint32_t * gpios) {

// }

// void moveLeftSideForward(volatile uint32_t * gpios) {

// }

// void stop(volatile uint32_t * gpios) {

// }

// void moveLeftSideBackward(volatile uint32_t * gpios) {

// }

int main() 
{
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

    // At this point, we have successfully mapped gpio registers to an array and have read/write access
    // to manipulate them according to our application needs
    volatile uint32_t * gpios = (volatile uint32_t *)gpio_reg_map;
    // init_gpio_pins(gpios);

    printf("\nClearing and initializing all motor GPIOs to output:");
    // Setup BL motor pin A
    printf("\n\tClearing and setting gpios[%i] to output and a shift of %i", (int)BL_RI_PIN_A, BL_SHIFT_A);
    static uint32_t set_bl_pina_gpio_output_mask = (0x0 | (1 << BL_SHIFT_A));
    // gpios[BL_RI_PIN_A] &= (0b000 << BL_SHIFT_A); // Clear current function by forcing bits to 000
    // gpios[BL_RI_PIN_A] |= (0b001 << BL_SHIFT_A); // Change GPIO to an output
    gpios[BL_RI_PIN_A] = set_bl_pina_gpio_output_mask;

    sleep(1);

    // Setup BL motor pin B
    printf("\n\tClearing and setting gpios[%i] to output and a shift of %i", (int)BL_RI_PIN_B, BL_SHIFT_B);
    static uint32_t set_bl_pinb_gpio_output_mask = (0x0 | (1 << BL_SHIFT_B));
    // gpios[BL_RI_PIN_B] &= (0b000 << BL_SHIFT_B); // Clear current function by forcing bits to 000
    // gpios[BL_RI_PIN_B] |= (0b001 << BL_SHIFT_B); // Change GPIO to an output
    gpios[BL_RI_PIN_B] = set_bl_pinb_gpio_output_mask;

    // printf("base address: 0x%0x\n", gpios);
    // printf("GPFSEL1 address: 0x%0x\n", gpios[1]);
    // printf("GPFSEL2 address: 0x%0x\n", gpios[2]);
/*
    // Setup BL motor pin B
    printf("\n\tClearing and setting gpios[%i] to output and a shift of %i", (int)FL_RI_PIN_A, FL_SHIFT_A);
    gpios[FL_RI_PIN_A] &= (0b000 << FL_SHIFT_A); // Clear current function by forcing bits to 000
    gpios[FL_RI_PIN_A] |= (0b001 << FL_SHIFT_A); // Change GPIO to an output

    // Setup BL motor pin B
    printf("\n\tClearing and setting gpios[%i] to output and a shift of %i", (int)FL_RI_PIN_B, FL_SHIFT_B);
    gpios[FL_RI_PIN_B] &= (0b000 << FL_SHIFT_B); // Clear current function by forcing bits to 000
    gpios[FL_RI_PIN_B] |= (0b001 << FL_SHIFT_B); // Change GPIO to an output

    printf("\nUsing GPIO_SET_REG %i", GPIO_SET_REG);
    printf("\nUsing GPIO_CLR_REG %i", GPIO_CLR_REG);
*/
    // Test...
    uint32_t count = 0;
    do {
        // moveLeftSideForward(gpios);
        printf("\nMoving Left Side Forward...");
        static uint32_t ldtf_set_mask = (0x0 | (1 << BL_MOTOR_PIN_A) | (1 << FL_MOTOR_PIN_A));
        static uint32_t ldtf_clr_mask = (0x0 | (1 << BL_MOTOR_PIN_B) | (1 << FL_MOTOR_PIN_B));
        printf("\n\tUsing set_mask = %i", ldtf_set_mask);
        printf("\n\tUsing clr_mask = %i", ldtf_clr_mask);
        // sleep(1);
        // gpios[GPIO_CLR_REG] = ldtf_clr_mask; // (1 << BL_MOTOR_PIN_B);
        // gpios[GPIO_SET_REG] = ldtf_set_mask; //(1 << BL_MOTOR_PIN_A); // Back left motor

        // gpios[(uint32_t)GPIO_SET_REG] |= (1 << FL_MOTOR_PIN_A); // Front left motor
        // gpios[(uint32_t)GPIO_CLR_REG] =  (1 << FL_MOTOR_PIN_B);

        // sleep(1);

        // stop(gpios)
        // printf("\nStopping Left Side Drive Train...");
        // static uint32_t ld_stop_mask = (0x0 | (1 << BL_MOTOR_PIN_A) | (1 << BL_MOTOR_PIN_B) | (1 << FL_MOTOR_PIN_A) | (1 << FL_MOTOR_PIN_B));
        // printf("\n\tUsing stop_mask = %i", ld_stop_mask);
        // sleep(1);
        // gpios[GPIO_CLR_REG] = ld_stop_mask; // (1 << BL_MOTOR_PIN_A); // Back left motor
        // gpios[(uint32_t)GPIO_CLR_REG] = (1 << BL_MOTOR_PIN_B);

        // gpios[(uint32_t)GPIO_CLR_REG] = (1 << FL_MOTOR_PIN_A); // Front left motor
        // gpios[(uint32_t)GPIO_CLR_REG] = (1 << FL_MOTOR_PIN_B);

        // sleep(1);

        // moveLeftSideBackward(gpios);
        // printf("\nMoving Left Side Backward...");
        // static uint32_t ldtb_set_mask = (0x0 | (1 << BL_MOTOR_PIN_B) | (1 << FL_MOTOR_PIN_B));
        // static uint32_t ldtb_clr_mask = (0x0 | (1 << BL_MOTOR_PIN_A) | (1 << FL_MOTOR_PIN_A));
        // printf("\n\tUsing set_mask = %i", ldtb_set_mask);
        // printf("\n\tUsing clr_mask = %i", ldtb_clr_mask);
        // sleep(1);
        // gpios[GPIO_CLR_REG] = ldtb_clr_mask; // (1 << BL_MOTOR_PIN_B);
        // gpios[GPIO_SET_REG] = ldtb_set_mask; // (1 << BL_MOTOR_PIN_A); // Back left motor
            
        // gpios[(uint32_t)GPIO_SET_REG] |= (1 << FL_MOTOR_PIN_A); // Front left motor
        // gpios[(uint32_t)GPIO_CLR_REG] =  (1 << FL_MOTOR_PIN_B);

        // sleep(1);
        count++;
    } while (count < 1);

    munmap(gpio_reg_map, GPIO_PUP_PDN_CNTRL_REG3);
    close(gpiomem_fd);
    printf("\n");

    return SUCCESS;
}