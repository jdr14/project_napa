import RPi.GPIO as gpio
from time import sleep

class PwmExperiment:
    def __init__(self):
        gpio.setmode(gpio.BCM) # Use the BCM GPIO layout
        self.pwm_freq = 1000 # 1kHz PWM frequency
        self.current_duty_cycle = 0 # Start at 0% duty cycle
        
        # PWM pins
        self.PWM_PIN_GPIO_12 = 12 # Pin 32 
        self.PWM_PIN_GPIO_18 = 18 # Pin 12

        gpio.setup(self.PWM_PIN_GPIO_12, gpio.OUT) # Pin 32
        gpio.setup(self.PWM_PIN_GPIO_18, gpio.OUT) # Pin 12
        self.pwm_12 = gpio.PWM(self.PWM_PIN_GPIO_12, self.pwm_freq)
        self.pwm_18 = gpio.PWM(self.PWM_PIN_GPIO_18, self.pwm_freq)
        
        # Set initial duty cycle
        self.pwm_12.start(0)
        self.pwm_18.start(0)
        
    def setDuty(self, value):
            self.pwm_12.ChangeDutyCycle(value)
            self.pwm_18.ChangeDutyCycle(value)
    
def main():
    exp = PwmExperiment() # Instantiate motors object which will set initial duty cycle to 0%
    
    # Conduct duty cycle sweep from 0-100% in increments of 5%
    duty_cycle = 5
    while duty_cycle <= 100:
        print(f"Setting Duty cycle to {duty_cycle}%")
        exp.setDuty(duty_cycle)
        sleep(2) # delay for 2 seconds so we can see the result on our oscilloscope
        duty_cycle += 5 # Increment by duty by 5% each cycle
        
    print("Resetting duty back to 0%")
    exp.setDuty(0)
    
if __name__ == '__main__':
    main()