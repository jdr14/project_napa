#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h> // Need for fd flags
#include <sys/mman.h> // mmap() defined in this header
#include <unistd.h> // Need to close fd and for sleep function

#include "BCM2711_ARM_GPIO.h"

#define SUCCESS (0)

#define VERBOSE (0) // Enable debug print statements

// These help with the bit shifting and arithmetic to properly modify the correct GPIO later in the program
#define FSEL_NUM_BITS (3)
#define GPIO_PINS_PER_REG (10)
#define BLOCK_SIZE (4096)

// These are the pins corresponding to the left side drive train
#define BL_MOTOR_PIN_A (17) // Pin 11
#define BL_MOTOR_PIN_B (22) // Pin 15
#define FL_MOTOR_PIN_A (24) // Pin 18
#define FL_MOTOR_PIN_B (23) // Pin 16

// These are the pins corresponding to the right side drive train
#define BR_MOTOR_PIN_A (26) // Pin 37
#define BR_MOTOR_PIN_B (27) // Pin 13
#define FR_MOTOR_PIN_A (5)  // Pin 29
#define FR_MOTOR_PIN_B (6)  // Pin 31

// PWM pins
#define BL_PWM0_0_PIN (12) // Pin 32 
#define BR_PWM1_1_PIN (13) // Pin 33
#define FL_PWM0_0_PIN (18) // Pin 12
#define FR_PWM1_1_PIN (19) // Pin 35

// PWM Shift registers (will have to use the alternate function options commented below)
#define BL_PWM0_SEL_SHIFT ((BL_PWM0_0_PIN % GPIO_PINS_PER_REG) * FSEL_NUM_BITS + 2) // Alternate function 0 (100)
#define BR_PWM1_SEL_SHIFT ((BR_PWM1_1_PIN % GPIO_PINS_PER_REG) * FSEL_NUM_BITS + 2) // Alternate function 0 (100)
#define FL_PWM0_SEL_SHIFT ((FL_PWM0_0_PIN % GPIO_PINS_PER_REG) * FSEL_NUM_BITS + 1) // Alternate function 5 (010)
#define FR_PWM1_SEL_SHIFT ((FR_PWM1_1_PIN % GPIO_PINS_PER_REG) * FSEL_NUM_BITS + 1) // Alternate function 5 (010)

// Define the Register Index which will be used to set the motor pins to output
#define BL_RI_PIN_A (BL_MOTOR_PIN_A / GPIO_PINS_PER_REG) // used
#define BL_RI_PIN_B (BL_MOTOR_PIN_B / GPIO_PINS_PER_REG) // used
// #define FL_RI_PIN_A (FL_MOTOR_PIN_A / GPIO_PINS_PER_REG)
// #define FL_RI_PIN_B (FL_MOTOR_PIN_B / GPIO_PINS_PER_REG)
// #define BR_RI_PIN_A (BR_MOTOR_PIN_A / GPIO_PINS_PER_REG)
// #define BR_RI_PIN_B (BR_MOTOR_PIN_B / GPIO_PINS_PER_REG)
#define FR_RI_PIN_A (FR_MOTOR_PIN_A / GPIO_PINS_PER_REG) // used
// #define FR_RI_PIN_B (FR_MOTOR_PIN_B / GPIO_PINS_PER_REG)

// Define the register set shifts
#define BL_SHIFT_A ((BL_MOTOR_PIN_A % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define BL_SHIFT_B ((BL_MOTOR_PIN_B % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define FL_SHIFT_A ((FL_MOTOR_PIN_A % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define FL_SHIFT_B ((FL_MOTOR_PIN_B % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define BR_SHIFT_A ((BR_MOTOR_PIN_A % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define BR_SHIFT_B ((BR_MOTOR_PIN_B % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define FR_SHIFT_A ((FR_MOTOR_PIN_A % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)
#define FR_SHIFT_B ((FR_MOTOR_PIN_B % GPIO_PINS_PER_REG) * FSEL_NUM_BITS)

// This Register will be used to set the pin high
#define GPIO_SET_REG (OFFSET_GPSET0 / GPIO_REG_SIZE)

// This Register will be used to set the pin low
#define GPIO_CLR_REG (OFFSET_CLR0 / GPIO_REG_SIZE)

// These are the masks bitwise or-ed with the set and clear registers to control the logicl level of the pins ultimately controlling polarity/direction of each motor
// I am using macros to define these up here so these definitions can be resolved entirely during the preprocessor stage of compile time
// saving us from using unnecessary cycles on an unchanging value
#define LEFT_SIDE_FORWARD_SET_MASK  ( (1 << BL_MOTOR_PIN_A) | (1 << FL_MOTOR_PIN_A) ) // Left Side Forward
#define LEFT_SIDE_FORWARD_CLR_MASK  ( (1 << BL_MOTOR_PIN_B) | (1 << FL_MOTOR_PIN_B) )

#define LEFT_SIDE_BACKWARD_SET_MASK ( (1 << BL_MOTOR_PIN_B) | (1 << FL_MOTOR_PIN_B) ) // Left Side Backward
#define LEFT_SIDE_BACKWARD_CLR_MASK ( (1 << BL_MOTOR_PIN_A) | (1 << FL_MOTOR_PIN_A) )

#define RIGHT_SIDE_FORWARD_SET_MASK  ( (1 << BR_MOTOR_PIN_A) | (1 << FR_MOTOR_PIN_A) ) // Right Side Forward
#define RIGHT_SIDE_FORWARD_CLR_MASK  ( (1 << BR_MOTOR_PIN_B) | (1 << FR_MOTOR_PIN_B) )

#define RIGHT_SIDE_BACKWARD_SET_MASK ( (1 << BR_MOTOR_PIN_B) | (1 << FR_MOTOR_PIN_B) ) // Right Side Backward
#define RIGHT_SIDE_BACKWARD_CLR_MASK ( (1 << BR_MOTOR_PIN_A) | (1 << FR_MOTOR_PIN_A) )

#define STOP_ALL_MOTORS_MASK ((1<<BL_MOTOR_PIN_A) | (1<<BL_MOTOR_PIN_B) | (1<<FL_MOTOR_PIN_A) | (1<<FL_MOTOR_PIN_B) | (1<<BR_MOTOR_PIN_A) | (1<<BR_MOTOR_PIN_B) | (1<<FR_MOTOR_PIN_A) | (1<<FR_MOTOR_PIN_B))

int handle_error(char * msg) 
{
    perror(msg); 
    return EXIT_FAILURE;
}

int init_gpio_pins(volatile uint32_t * gpios) 
{
    printf("\nClearing and initializing all motor GPIOs to output:");
    
    // Set functionality for motor pins and all the PWM control pins
    gpios[(uint32_t)FR_RI_PIN_A] |= (1 << FR_SHIFT_A) | (1 << FR_SHIFT_B); // Should be register 0
    gpios[(uint32_t)BL_RI_PIN_A] |= (1 << BL_SHIFT_A) | (1 << BL_PWM0_SEL_SHIFT) | (1 << BR_PWM1_SEL_SHIFT) | (1 << FL_PWM0_SEL_SHIFT) | (1 << FR_PWM1_SEL_SHIFT); // Should be register 1
    gpios[(uint32_t)BL_RI_PIN_B] |= (1 << BL_SHIFT_B) | (1 << FL_SHIFT_A) | (1 << FL_SHIFT_B) | (1 << BR_SHIFT_A) | (1 << BR_SHIFT_B); // Should be register 2
}

void stop(volatile uint32_t * gpios) 
{
    printf("\nStopping Left Side Drive Train...");
    gpios[GPIO_CLR_REG] = STOP_ALL_MOTORS_MASK;
}

void moveForward(volatile uint32_t * gpios)
{
    printf("\nMoving Left Side Forward...");
    gpios[GPIO_CLR_REG] = LEFT_SIDE_FORWARD_CLR_MASK;
    gpios[GPIO_SET_REG] = LEFT_SIDE_FORWARD_SET_MASK;

    printf("\nMoving Right Side Forward...");
    gpios[GPIO_CLR_REG] = RIGHT_SIDE_FORWARD_CLR_MASK;
    gpios[GPIO_SET_REG] = RIGHT_SIDE_FORWARD_SET_MASK;
}

void moveBackward(volatile uint32_t * gpios)
{
    printf("\nMoving Left Side Backward...");
    gpios[GPIO_CLR_REG] = LEFT_SIDE_BACKWARD_CLR_MASK;
    gpios[GPIO_SET_REG] = LEFT_SIDE_BACKWARD_SET_MASK;

    printf("\nMoving Right Side Backward...");
    gpios[GPIO_CLR_REG] = RIGHT_SIDE_BACKWARD_CLR_MASK;
    gpios[GPIO_SET_REG] = RIGHT_SIDE_BACKWARD_SET_MASK;
}

void turnLeft(volatile uint32_t * gpios) 
{
    printf("\nMoving Left Side Backward...");
    gpios[GPIO_CLR_REG] = LEFT_SIDE_BACKWARD_CLR_MASK;
    gpios[GPIO_SET_REG] = LEFT_SIDE_BACKWARD_SET_MASK;

    printf("\nMoving Right Side Forward...");
    gpios[GPIO_CLR_REG] = RIGHT_SIDE_FORWARD_CLR_MASK;
    gpios[GPIO_SET_REG] = RIGHT_SIDE_FORWARD_SET_MASK;
}

void turnRight(volatile uint32_t * gpios)
{
    printf("\nMoving Left Side Forward...");
    gpios[GPIO_CLR_REG] = LEFT_SIDE_FORWARD_CLR_MASK;
    gpios[GPIO_SET_REG] = LEFT_SIDE_FORWARD_SET_MASK;

    printf("\nMoving Right Side Backward...");
    gpios[GPIO_CLR_REG] = RIGHT_SIDE_BACKWARD_CLR_MASK;
    gpios[GPIO_SET_REG] = RIGHT_SIDE_BACKWARD_SET_MASK;
}

int init_pwm_registers(volatile uint32_t * pwm) {
    printf("\nPWM 0 index = %i | PWM 1 index = %i", PWM0_CTL_REG_INDEX, PWM1_CTL_REG_INDEX);
    pwm[PWM0_CTL_REG_INDEX] = CTL_REG_VALUE;
    pwm[PWM1_CTL_REG_INDEX] = CTL_REG_VALUE;
}

void setSpeed(volatile uint32_t * pwm) {
    pwm[PWM0_DAT1_REG_INDEX] = DUTY_CYCLE_50P;
    pwm[PWM0_DAT2_REG_INDEX] = DUTY_CYCLE_50P;
    pwm[PWM1_DAT1_REG_INDEX] = DUTY_CYCLE_100P;
    pwm[PWM1_DAT2_REG_INDEX] = DUTY_CYCLE_100P;
}

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
    volatile uint32_t * gpios = (volatile uint32_t *)mmap(
        NULL,                    // Allow kernel to decide the virtual base address
        BLOCK_SIZE,         // This is the size of the GPIO memory block
        PROT_READ | PROT_WRITE,  // Enable read and write access
        MAP_SHARED,              // Updates to mapping are visible by other processes, which is what we want with GPIO
        gpiomem_fd,              // FD we created above
        GPIO_REG_BASE_ADDRESS);  // The start address of the GPIO memory block

    // At this point, we have successfully mapped gpio registers to an array and have read/write access
    // to manipulate them according to our application needs 
    init_gpio_pins(gpios);

    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        return handle_error("open /dev/mem failed");
    }

    volatile uint32_t * pwm = (volatile uint32_t *)mmap(
        NULL,
        BLOCK_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        PWM0_REG_BASE_ADDRESS);
    if (pwm == MAP_FAILED) {
        return handle_error("Failed to map pwm");
    }

    // volatile uint32_t * pwm1 = (volatile uint32_t *)mmap(
    //     NULL,
    //     BLOCK_SIZE,
    //     PROT_READ | PROT_WRITE,
    //     MAP_SHARED,
    //     mem_fd,
    //     PWM1_REG_BASE_ADDRESS);
    // if (pwm1 == MAP_FAILED) {
    //     return handle_error("Failed to map pwm1");
    // }

    init_pwm_registers(pwm);

    // Test...
    uint32_t count = 0;
    do {
        moveForward(gpios);

        sleep(2);
        stop(gpios);
        sleep(1);
        // setSpeed(pwm0, pwm1);

        moveBackward(gpios);

        sleep(2);
        stop(gpios);
        sleep(1);

        turnRight(gpios);

        sleep(2);
        stop(gpios);
        sleep(1);

        turnLeft(gpios);

        sleep(2);
        count++;
    } while (count < 1);

    stop(gpios);

    munmap((void *)gpios, BLOCK_SIZE);
    close(gpiomem_fd);

    munmap((void *)pwm, BLOCK_SIZE);
    close(mem_fd);
    printf("\n");

    return SUCCESS;
}