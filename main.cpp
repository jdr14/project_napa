#include <iostream>

#include <pigpio.h>

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
#define BL_PWM0_0_PIN (12) // Pin 32 (Needs Alt func 0)
#define BR_PWM1_1_PIN (13) // Pin 33 (Needs Alt func 0)
#define FL_PWM0_0_PIN (18) // Pin 12 (Needs Alt func 5)
#define FR_PWM1_1_PIN (19) // Pin 35 (Needs Alt func 5)
// #define BL_PWM0_0_PIN (32) // Pin 32 (Needs Alt func 0)
// #define BR_PWM1_1_PIN (33) // Pin 33 (Needs Alt func 0)
// #define FL_PWM0_0_PIN (12) // Pin 12 (Needs Alt func 5)
// #define FR_PWM1_1_PIN (35) // Pin 35 (Needs Alt func 5)

// void setup() 
// {
//     // if (wiringPiSetup() == -1) {
//     if (wiringPiSetupGpio() == -1) {
//         std::cerr << "Setup Failed" << std::endl;
//         exit(1);
//     }
    
//     // These are the pins corresponding to the left side drive train
//     pinMode(BL_MOTOR_PIN_A, OUTPUT);
//     pinMode(BL_MOTOR_PIN_B, OUTPUT);
//     pinMode(FL_MOTOR_PIN_A, OUTPUT);
//     pinMode(FL_MOTOR_PIN_B, OUTPUT);
    
//     // These are the pins corresponding to the left side drive train
//     pinMode(BR_MOTOR_PIN_A, OUTPUT);
//     pinMode(BR_MOTOR_PIN_B, OUTPUT);
//     pinMode(FR_MOTOR_PIN_A, OUTPUT);
//     pinMode(FR_MOTOR_PIN_B, OUTPUT);
    
//     // PWM pin setup
//     pinMode(BL_PWM0_0_PIN, PWM_OUTPUT);
//     pinMode(BR_PWM1_1_PIN, PWM_OUTPUT);
//     pinMode(FL_PWM0_0_PIN, PWM_OUTPUT);
//     pinMode(FR_PWM1_1_PIN, PWM_OUTPUT);
    
//     pwmSetRange(1024);
//     pwmSetClock(32);
// }

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio!" << std::endl;
        return 1;
    }

    int pwmPin = 18;  // GPIO18 (Hardware PWM0)
    int frequency = 1000; // 1 kHz
    int dutyCycle = 128000; // 50% (Range: 0-1,000,000)

    gpioHardwarePWM(pwmPin, frequency, dutyCycle);

    std::cout << "PWM running on GPIO " << pwmPin << std::endl;
    getchar(); // Wait for user input
    gpioTerminate();
    return 0;
}

// int main(void)
// {
//     setup();
//     std::cout << "Robot Napa is awake!\n";
    
//     digitalWrite(BL_MOTOR_PIN_A, HIGH);
//     digitalWrite(BL_MOTOR_PIN_B, LOW);
    
//     digitalWrite(FL_MOTOR_PIN_A, HIGH);
//     digitalWrite(FL_MOTOR_PIN_B, LOW);

//     digitalWrite(BR_MOTOR_PIN_A, LOW);
//     digitalWrite(BR_MOTOR_PIN_B, HIGH);
    
//     digitalWrite(FR_MOTOR_PIN_A, HIGH);
//     digitalWrite(FR_MOTOR_PIN_B, LOW);
    
//     // delay(100);
    
//     int speed;
//     for (int i = 0; i < 1; i++) {
//         for(speed = 0; speed < 1024; ++speed) {
//             pwmWrite(BL_PWM0_0_PIN, speed);
//             pwmWrite(BR_PWM1_1_PIN, speed);
//             pwmWrite(FL_PWM0_0_PIN, speed);
//             pwmWrite(FR_PWM1_1_PIN, speed);
//             delay(1);
//         }
//         for(speed = 1023; speed >= 0; --speed) {
//             pwmWrite(BL_PWM0_0_PIN, speed);
//             pwmWrite(BR_PWM1_1_PIN, speed);
//             pwmWrite(FL_PWM0_0_PIN, speed);
//             pwmWrite(FR_PWM1_1_PIN, speed);
//             delay(1);
//         }
//     }
    
//     digitalWrite(BL_MOTOR_PIN_A, LOW);
//     digitalWrite(BL_MOTOR_PIN_B, LOW);
    
//     digitalWrite(FL_MOTOR_PIN_A, LOW);
//     digitalWrite(FL_MOTOR_PIN_B, LOW);

//     digitalWrite(BR_MOTOR_PIN_A, LOW);
//     digitalWrite(BR_MOTOR_PIN_B, LOW);
    
//     digitalWrite(FR_MOTOR_PIN_A, LOW);
//     digitalWrite(FR_MOTOR_PIN_B, LOW);
    
//     return 0;
// }