from multiprocessing import Process, Queue
import RPi.GPIO as gpio
from evdev import InputDevice, categorize, ecodes
from time import sleep
import asyncio

# Through trial and error of the following, I've been able to figure out the pro controller mappings
# Button Pad
# B  --> code = 304 | type = 1 | value = 1
# A  --> code = 305 | type = 1 | value = 1
# X  --> code = 307 | type = 1 | value = 1
# Y  --> code = 308 | type = 1 | value = 1
#
# Triggers
# L  --> code = 310 | type = 1 | value = 1
# R  --> code = 311 | type = 1 | value = 1
# XL --> code = 312 | type = 1 | value = 1
# XR --> code = 313 | type = 1 | value = 1
#
# Joystick Button Press
# Left  Joystick --> code = 317 | type = 1 | value = 1
# RIght Joystick --> code = 318 | type = 1 | value = 1
#
# Volume buttons
# - --> code = 314 | type = 1 | value = 1
# + --> code = 315 | type = 1 | value = 1
# 
# Select and Home
# Home   --> code = 316 | type = 1 | value = 1
# Select --> code = 309 | type = 1 | value = 1

# JOYSTICK Mapping
# Left joystick all the way forward --> code = 1 | value = -32K
# Left joystick all the way backward --> code = 1 | value = 32K
# Left joystick all the way right --> code = 0 | value = 32K
# Left joystick all the way left --> code = 0 | value = -32K
#
class Controller:
    def __init__(self, thread_safe_controller_queue):
        self.ipc_queue = thread_safe_controller_queue
        
        # Find the device path. You might need to check /dev/input/event* for the correct one.
        self.controller = InputDevice('/dev/input/event4') # Replace X with the correct event number
        sleep(2)
        
        print(f"Successfully connected to {self.controller.name}")
        print(self.controller.phys)
        
    async def read_controller(self):
        async for event in self.controller.async_read_loop():
            if event.type == ecodes.EV_ABS:
                # print(f"type = {event.type} | code = {event.code} | value = {event.value}")
                self.ipc_queue.put([event.type, event.code, event.value])
            
    def controller_input(self):
        asyncio.run(self.read_controller())

class Motors:
    def __init__(self, thread_safe_controller_queue):
        # Using a thread safe queue as the method for inter process communication
        self.ipc_queue = thread_safe_controller_queue
        
        # These are the pins corresponding to the left side drive train
        self.BL_MOTOR_PIN_A = 17 # Pin 11
        self.BL_MOTOR_PIN_B = 22 # Pin 15
        self.FL_MOTOR_PIN_A = 24 # Pin 18
        self.FL_MOTOR_PIN_B = 23 # Pin 16
        
        # These are the pins corresponding to the right side drive train
        self.BR_MOTOR_PIN_A = 26 # Pin 37
        self.BR_MOTOR_PIN_B = 27 # Pin 13
        self.FR_MOTOR_PIN_A = 5  # Pin 29
        self.FR_MOTOR_PIN_B = 6  # Pin 31
        
        # PWM pins
        self.BL_PWM0_0_PIN = 12 # Pin 32 
        self.BR_PWM1_1_PIN = 13 # Pin 33
        self.FL_PWM0_0_PIN = 18 # Pin 12
        self.FR_PWM1_1_PIN = 19 # Pin 35

    def forward(self):
        gpio.output(self.FL_MOTOR_PIN_A, gpio.HIGH)
        gpio.output(self.FL_MOTOR_PIN_B, gpio.LOW)
        
    def run(self):
        while True:
            eType, eCode, eValue = self.ipc_queue.get()
            print(f"eType = {eType} | eCode = {eCode} | eValue = {eValue}")
    

def main():
    ctl_queue = Queue()
    ctlr = Controller(ctl_queue)
    motors = Motors(ctl_queue)
    # parent_conn, child_conn = Pipe()
    ctlProcess = Process(target=ctlr.controller_input)
    motorProcess = Process(target=motors.run)
    ctlProcess.start()
    motorProcess.start()
    ctlProcess.join()
    motorProcess.join()

if __name__ == '__main__':
    main()