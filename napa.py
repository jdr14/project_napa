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
    def __init__(self, thread_safe_controller_queue):
        self.ipc_queue = thread_safe_controller_queue
        
        # Find the device path. You might need to check /dev/input/event* for the correct one.
        self.controller = InputDevice('/dev/input/event4') # Replace X with the correct event number
        time.sleep(2)
        
        print(f"Successfully connected to {self.controller.name}")
        print(self.controller.phys)
        
    async def read_controller(self):
    # def read_controller(self):
        async for event in self.controller.async_read_loop():
            # await asyncio.sleep(0.25) # sleep for 250 ms
            # event = self.controller.read_one()
            if event and event.type == ecodes.EV_ABS:
                # print(f"type = {event.type} | code = {event.code} | value = {event.value}")
                self.ipc_queue.put([event.type, event.code, event.value])
        # threading.Timer(0.1, self.read_controller).start()
        # time.sleep(0.1)
            
    def controller_input(self):
        asyncio.run(self.read_controller())
        # threading.Timer(0.25, self.read_controller).start()
        # self.timerThread = threading.Thread(target=self.read_controller)
        # self.timerThread.daemon = True
        # self.timerThread.start()

class Motors:
    def __init__(self, thread_safe_controller_queue):
        # Using a thread safe queue as the method for inter process communication
        self.ipc_queue = thread_safe_controller_queue
        
        gpio.setmode(gpio.BCM)
        self.pwm_freq = 1000 # 1kHz PWM frequency
        self.drift_offset = 2000 # This accounts for the controller joystick drift/noise
        self.duty_cycle_increment = 5 # 5% duty cycle increments 
        
        self.direction = "stop"
        self.current_duty_cycle = 0
        
        # Left side drive train pins
        self.BL_MOTOR_PIN_A = 17 # Pin 11
        self.BL_MOTOR_PIN_B = 22 # Pin 15
        self.FL_MOTOR_PIN_A = 24 # Pin 18
        self.FL_MOTOR_PIN_B = 23 # Pin 16
        
        # Right side drive train pins
        self.BR_MOTOR_PIN_A = 26 # Pin 37
        self.BR_MOTOR_PIN_B = 27 # Pin 13
        self.FR_MOTOR_PIN_A = 5  # Pin 29
        self.FR_MOTOR_PIN_B = 6  # Pin 31
        
        # PWM pins
        self.BL_PWM0_0_PIN = 12 # Pin 32 
        self.BR_PWM1_1_PIN = 13 # Pin 33
        self.FL_PWM0_0_PIN = 18 # Pin 12
        self.FR_PWM1_1_PIN = 19 # Pin 35
        
    def left_side_dt_forward(self):
        gpio.output(self.FL_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.FL_MOTOR_PIN_B, gpio.HIGH)
        gpio.output(self.BL_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.BL_MOTOR_PIN_B, gpio.HIGH)

    def right_side_dt_forward(self):
        gpio.output(self.FR_MOTOR_PIN_A, gpio.HIGH)
        gpio.output(self.FR_MOTOR_PIN_B, gpio.LOW)
        gpio.output(self.BR_MOTOR_PIN_A, gpio.HIGH)
        gpio.output(self.BR_MOTOR_PIN_B, gpio.LOW)
        
    def _left_side_dt_backward(self):
        gpio.output(self.FL_MOTOR_PIN_A, gpio.HIGH)
        gpio.output(self.FL_MOTOR_PIN_B, gpio.LOW)
        gpio.output(self.BL_MOTOR_PIN_A, gpio.HIGH)
        gpio.output(self.BL_MOTOR_PIN_B, gpio.LOW)
        
    def _right_side_dt_backward(self):
        gpio.output(self.FR_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.FR_MOTOR_PIN_B, gpio.HIGH)
        gpio.output(self.BR_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.BR_MOTOR_PIN_B, gpio.HIGH)
        
    def stop_left(self):
        gpio.output(self.FL_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.FL_MOTOR_PIN_B, gpio.LOW)
        gpio.output(self.BL_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.BL_MOTOR_PIN_B, gpio.LOW)
        
    def stop_right(self):
        gpio.output(self.FR_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.FR_MOTOR_PIN_B, gpio.LOW)
        gpio.output(self.BR_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.BR_MOTOR_PIN_B, gpio.LOW)
        
    def setSpeed(self, code, value):
        if not code or not value:
            return
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
            if code == 1:
                self.bl_pwm.ChangeDutyCycle(self.current_duty_cycle)
                self.fl_pwm.ChangeDutyCycle(self.current_duty_cycle)
            elif code == 4:
                self.br_pwm.ChangeDutyCycle(self.current_duty_cycle)
                self.fr_pwm.ChangeDutyCycle(self.current_duty_cycle)
        
    def setDirection(self, code, value):
        if not code or not value: # Nonetype guard
            return
        if code == 1 and math.fabs(value) < self.drift_offset:
            self.stop_left()
        elif code == 4 and math.fabs(value) < self.drift_offset:
            self.stop_right()
        elif code == 1 and value < (-1 * self.drift_offset):
            self.left_side_dt_forward()
        elif code == 1 and value > self.drift_offset:
            self._left_side_dt_backward()
        elif code == 4 and value < (-1 * self.drift_offset):
            self.right_side_dt_forward()
        elif code == 4 and value > self.drift_offset:
            self._right_side_dt_backward()
    
    def _setup(self):
        # Left side drive train pins
        gpio.setup(self.BL_MOTOR_PIN_A, gpio.OUT) 
        gpio.setup(self.BL_MOTOR_PIN_B, gpio.OUT)
        gpio.setup(self.FL_MOTOR_PIN_A, gpio.OUT)
        gpio.setup(self.FL_MOTOR_PIN_B, gpio.OUT)
        
        # Right side drive train pins
        gpio.setup(self.BR_MOTOR_PIN_A, gpio.OUT)
        gpio.setup(self.BR_MOTOR_PIN_B, gpio.OUT)
        gpio.setup(self.FR_MOTOR_PIN_A, gpio.OUT)
        gpio.setup(self.FR_MOTOR_PIN_B, gpio.OUT)
        
        # PWM pins
        gpio.setup(self.BL_PWM0_0_PIN, gpio.OUT)
        gpio.setup(self.FL_PWM0_0_PIN, gpio.OUT)
        gpio.setup(self.BR_PWM1_1_PIN, gpio.OUT)
        gpio.setup(self.FR_PWM1_1_PIN, gpio.OUT)
        
        # Set PWM Frequency
        self.bl_pwm = gpio.PWM(self.BL_PWM0_0_PIN, self.pwm_freq)
        self.fl_pwm = gpio.PWM(self.FL_PWM0_0_PIN, self.pwm_freq)
        self.br_pwm = gpio.PWM(self.BR_PWM1_1_PIN, self.pwm_freq)
        self.fr_pwm = gpio.PWM(self.FR_PWM1_1_PIN, self.pwm_freq)
        
        # Set intiial duty cycle
        self.bl_pwm.start(self.current_duty_cycle)
        self.fl_pwm.start(self.current_duty_cycle)
        self.br_pwm.start(self.current_duty_cycle)
        self.fr_pwm.start(self.current_duty_cycle)
        
    def run(self):
        self._setup()
        
        eType, eCode, eValue = (None, None, None)
        
        def _mc_callback():
            while True:
                if eCode == 1 or eCode == 4:
                    self.setDirection(eCode, eValue)
                    self.setSpeed(eCode, eValue)
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
    
    ctl_queue = Queue()
    ctlr = Controller(ctl_queue)
    motors = Motors(ctl_queue)
    ctlProcess = Process(target=ctlr.controller_input)
    motorProcess = Process(target=motors.run)
    
    # Start and append the bluetooth controller process
    ctlProcess.start()
    procList.append(ctlProcess)
    
    # Start and append the motor control process
    motorProcess.start()
    procList.append(motorProcess)
    
    # Keep main process going
    try:
        while True:
            time.sleep(1)
    except Exception as e:
        print(repr(e))
        signal_handler(None, None)

if __name__ == '__main__':
    main()