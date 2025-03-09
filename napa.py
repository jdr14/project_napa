from multiprocessing import Process, Queue
import RPi.GPIO as gpio
from evdev import InputDevice, categorize, ecodes
import time
import threading
import signal
import asyncio
import math
import sys

procList = []

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
    def __init__(self, thread_safe_ctl_q_left, thread_safe_ctl_q_right):
        self.ipc_q_left = thread_safe_ctl_q_left
        self.ipc_q_right = thread_safe_ctl_q_right
        
        # Find the device path. You might need to check /dev/input/event* for the correct one.
        self.controller = InputDevice('/dev/input/event4') # Replace X with the correct event number
        
        print(f"Successfully connected to {self.controller.name}")
        print(self.controller.phys)
        
    async def read_controller(self):
        async for event in self.controller.async_read_loop():
            if event and event.type == ecodes.EV_ABS:
                if event.code == 1: # left joystick (0 and 1)
                    self.ipc_q_left.put([event.type, event.code, event.value])
                elif event.code == 4: # right joystick (3 and 4)
                    self.ipc_q_right.put([event.type, event.code, event.value])
            
    def controller_input(self):
        asyncio.run(self.read_controller())

class Motors:
    """
    left  side pins (in order): fpA (17), fpB (22), bpA (24), bpB (23), pw0 (12), pw1 (18)
    right side pins (in order): fpA (27), fpB (26), bpA (6),  bpB (5),  pw0 (13), pw1 (19)
    """
    def __init__(self, thread_safe_controller_queue, fpA, fpB, bpA, bpB, pw0, pw1):
        # Using a thread safe queue as the method for inter process communication
        self.ipc_queue = thread_safe_controller_queue
        
        gpio.setmode(gpio.BCM)
        self.pwm_freq = 1000 # 1kHz PWM frequency
        self.drift_offset = 2000 # This accounts for the controller joystick drift/noise
        self.duty_cycle_increment = 5 # 5% duty cycle increments 
        self.current_duty_cycle = 0
        
        self.MOTOR_PIN_FRONT_A = fpA # Pin 11
        self.MOTOR_PIN_FRONT_B = fpB # Pin 15
        self.MOTOR_PIN_BACK_A = bpA # Pin 18
        self.MOTOR_PIN_BACK_B = bpB # Pin 16
        
        # PWM pins
        self.PWM_PIN_0 = pw0 # Pin 32 
        self.PWM_PIN_1 = pw1 # Pin 33
        
    def forward(self):
        gpio.output(self.MOTOR_PIN_FRONT_A, gpio.LOW)
        gpio.output(self.MOTOR_PIN_FRONT_B, gpio.HIGH)
        gpio.output(self.MOTOR_PIN_BACK_A, gpio.LOW)
        gpio.output(self.MOTOR_PIN_BACK_B, gpio.HIGH)
        
    def backward(self):
        gpio.output(self.MOTOR_PIN_FRONT_A, gpio.HIGH)
        gpio.output(self.MOTOR_PIN_FRONT_B, gpio.LOW)
        gpio.output(self.MOTOR_PIN_BACK_A, gpio.HIGH)
        gpio.output(self.MOTOR_PIN_BACK_B, gpio.LOW)
        
    def stop(self):
        gpio.output(self.MOTOR_PIN_FRONT_A, gpio.LOW)
        gpio.output(self.MOTOR_PIN_FRONT_B, gpio.LOW)
        gpio.output(self.MOTOR_PIN_BACK_A, gpio.LOW)
        gpio.output(self.MOTOR_PIN_BACK_B, gpio.LOW)
        
    def setSpeed(self, value):
        duty_cycle_percent = 0
        # Have a small range of buffer to account for joystick drift (if it within this range, we consider the effective duty_cycle to be 0%)
        if math.fabs(value) > self.drift_offset:
            # Translate the raw joystick value to the 0-100% duty cycle for PWM control
            duty_cycle_percent = int(math.floor( (math.fabs(math.fabs(value) - self.drift_offset) * 3.15) / (1000 * self.duty_cycle_increment) )) * 5 + 5
            if duty_cycle_percent > 100:
                duty_cycle_percent = 100
            elif duty_cycle_percent < 10:
                duty_cycle_percent = 0
                
        if duty_cycle_percent != self.current_duty_cycle: # Only bother updating if there is a change
            self.current_duty_cycle = duty_cycle_percent    
            print(f"Setting Duty Cycle to {self.current_duty_cycle}%")
            self.pwm_0.ChangeDutyCycle(self.current_duty_cycle)
            self.pwm_1.ChangeDutyCycle(self.current_duty_cycle)
        
    def setDirection(self, value):
        if math.fabs(value) < self.drift_offset:
            self.stop()
        elif value < (-1 * self.drift_offset):
            self.forward()
        elif value > self.drift_offset:
            self.backward()
    
    def _setup(self):
        # Left side drive train pins
        gpio.setup(self.MOTOR_PIN_FRONT_A, gpio.OUT) 
        gpio.setup(self.MOTOR_PIN_FRONT_B, gpio.OUT)
        gpio.setup(self.MOTOR_PIN_BACK_A, gpio.OUT)
        gpio.setup(self.MOTOR_PIN_BACK_B, gpio.OUT)
        
        # PWM pins
        gpio.setup(self.PWM_PIN_0, gpio.OUT)
        gpio.setup(self.PWM_PIN_1, gpio.OUT)
        
        # Set PWM Frequency
        self.pwm_0 = gpio.PWM(self.PWM_PIN_0, self.pwm_freq)
        self.pwm_1 = gpio.PWM(self.PWM_PIN_1, self.pwm_freq)
        
        # Set intiial duty cycle
        self.pwm_0.start(self.current_duty_cycle)
        self.pwm_1.start(self.current_duty_cycle)
        
    def run(self):
        self._setup()
        
        eType, eCode, eValue = (0, 0, 0)
        
        def _mc_callback():
            while True:
                # if eCode == 1 or eCode == 4:
                self.setDirection(eValue)
                self.setSpeed(eValue)
                time.sleep(0.1) # Delay for stability
            
        mc_worker = threading.Thread(target=_mc_callback)
        mc_worker.daemon = True
        mc_worker.start()
            
        while True:
            eType, eCode, eValue = self.ipc_queue.get()
            # print(f"eType = {eType} | eCode = {eCode} | eValue = {eValue}")

"""
Handle signals and gracefully shutdown before exit
"""
def signal_handler(sgnl, _):
    gpio.cleanup()
    for proc in procList:
        if proc.is_alive():
            proc.terminate()
            print(f"Process \"{proc.pid}\" terminated")
    
    for proc in procList:
        proc.join(timeout=2) # Timeout to prevent hang
        print(f"Process \"{proc.pid}\" joined")
    sys.exit(0)
    
def main():
    # Register our signal handlers for the desired signal
    signal.signal(signal.SIGINT,  signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # left  side pins (in order): fpA (17), fpB (22), bpA (24), bpB (23), pw0 (12), pw1 (18)
    # right side pins (in order): fpA (27), fpB (26), bpA (6),  bpB (5),  pw0 (13), pw1 (19)
    
    # Use thread safe queues as our method of IPC
    ctl_queue_left = Queue()
    ctl_queue_right = Queue()
    
    # Create the controller and drive train instances
    ctlr = Controller(ctl_queue_left, ctl_queue_right)
    motors_left = Motors(ctl_queue_left, 17, 22, 24, 23, 12, 18) # pass in relevant GPIO/PWM pins used for left side drive train
    motors_right = Motors(ctl_queue_right, 27, 26, 6, 5, 13, 19) # pass in relevant GPIO/PWM pins used for right side drive train
    
    # Spin up the processes
    ctlrProcess = Process(target=ctlr.controller_input)
    leftDriveTrainProcess = Process(target=motors_left.run)
    rightDriveTrainProcess = Process(target=motors_right.run)
    
    # Start and append the bluetooth controller process
    ctlrProcess.start()
    procList.append(ctlrProcess)
    
    # Start and append the motor control process
    leftDriveTrainProcess.start()
    procList.append(leftDriveTrainProcess)
    rightDriveTrainProcess.start()
    procList.append(rightDriveTrainProcess)
    
    # Keep main process going
    try:
        while True:
            time.sleep(1)
    except Exception as e:
        print(repr(e))
        signal_handler(None, None)

if __name__ == '__main__':
    main()