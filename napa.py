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
        
        # These are the pins corresponding to the left side drive train
        self.BL_MOTOR_PIN_A = 17 # Pin 11
        self.BL_MOTOR_PIN_B = 22 # Pin 15
        self.FL_MOTOR_PIN_A = 24 # Pin 18
        self.FL_MOTOR_PIN_B = 23 # Pin 16
        
        gpio.setup(self.BL_MOTOR_PIN_A, gpio.OUT) # Pin 11
        gpio.setup(self.BL_MOTOR_PIN_B, gpio.OUT) # Pin 15
        gpio.setup(self.FL_MOTOR_PIN_A, gpio.OUT) # Pin 18
        gpio.setup(self.FL_MOTOR_PIN_B, gpio.OUT) # Pin 16
        
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

        # gpio.setup(self.BL_PWM0_0_PIN, gpio.OUT) # Pin 32
        # gpio.setup(self.FL_PWM0_0_PIN, gpio.OUT) # Pin 12
        # self.bl_pwm = gpio.PWM(self.BL_PWM0_0_PIN, self.pwm_freq)
        # self.fl_pwm = gpio.PWM(self.FL_PWM0_0_PIN, self.pwm_freq)
        # print(f"Starting PWM pins at a duty cycle of {self.current_duty_cycle}")
        # self.bl_pwm.start(self.current_duty_cycle)
        # self.fl_pwm.start(self.current_duty_cycle)

    def forward(self):
        gpio.output(self.FL_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.FL_MOTOR_PIN_B, gpio.HIGH)
        # gpio.output(self.BL_MOTOR_PIN_A, gpio.HIGH)
        # gpio.output(self.BL_MOTOR_PIN_B, gpio.LOW)
        
    def backward(self):
        gpio.output(self.FL_MOTOR_PIN_A, gpio.HIGH)
        gpio.output(self.FL_MOTOR_PIN_B, gpio.LOW)
        # gpio.output(self.BL_MOTOR_PIN_A, gpio.LOW)
        # gpio.output(self.BL_MOTOR_PIN_B, gpio.HIGH)
        
    def stop(self):
        gpio.output(self.FL_MOTOR_PIN_A, gpio.LOW)
        gpio.output(self.FL_MOTOR_PIN_B, gpio.LOW)
        # gpio.output(self.BL_MOTOR_PIN_A, gpio.LOW)
        # gpio.output(self.BL_MOTOR_PIN_B, gpio.LOW)
        
    def setSpeed(self, value):
        # Translate the raw joystick value to the 0-100% duty cycle for PWM control
        if math.fabs(value) < self.drift_offset and self.current_duty_cycle != 0: # Have a small range of buffer to account for joystick drift
            print(f"Setting Duty Cycle to 0%")
            self.current_duty_cycle = 0
            self.bl_pwm.ChangeDutyCycle(self.current_duty_cycle)
            self.fl_pwm.ChangeDutyCycle(self.current_duty_cycle)
            return
        duty_cycle_percent = int(math.floor( (math.fabs(math.fabs(value) - self.drift_offset) * 3.15) / (1000 * self.duty_cycle_increment) )) * 5 + 5
        if duty_cycle_percent > 100:
            duty_cycle_percent = 100
        if duty_cycle_percent < 10:
            duty_cycle_percent = 0
        if duty_cycle_percent != self.current_duty_cycle:
            print(f"Setting Duty Cycle to {duty_cycle_percent}%")
            self.current_duty_cycle = duty_cycle_percent
            self.bl_pwm.ChangeDutyCycle(self.current_duty_cycle)
            self.fl_pwm.ChangeDutyCycle(self.current_duty_cycle)
        
    def setDirection(self, value):
        if math.fabs(value) < self.drift_offset and self.direction != "stop":
            print("Stopping")
            self.direction = "stop"
            self.stop()
        elif value < (-1 * self.drift_offset) and self.direction != "forward":
            print("Moving Forward")
            self.direction = "forward"
            self.forward()
        elif value > self.drift_offset and self.direction != "backward":
            print("Moving Backward")
            self.direction = "backward"
            self.backward()
        
    def run(self):    
        eType, eCode, eValue = (None, None, None)
        
        gpio.setup(self.BL_PWM0_0_PIN, gpio.OUT) # Pin 32
        gpio.setup(self.FL_PWM0_0_PIN, gpio.OUT) # Pin 12
        self.bl_pwm = gpio.PWM(self.BL_PWM0_0_PIN, self.pwm_freq)
        self.fl_pwm = gpio.PWM(self.FL_PWM0_0_PIN, self.pwm_freq)
        self.bl_pwm.start(self.current_duty_cycle)
        self.fl_pwm.start(self.current_duty_cycle)
            
        def _mc_callback():
            while True:
                if eCode == 1:
                    self.setDirection(eValue)
                    self.setSpeed(eValue)
                time.sleep(0.1) # Delay for stability
            
        mc_worker = threading.Thread(target=_mc_callback)
        mc_worker.daemon = True
        mc_worker.start()
            
        while True:
            eType, eCode, eValue = self.ipc_queue.get()
        
        # while True:
            # eType, eCode, eValue = self.ipc_queue.get()
            # print(f"eType = {eType} | eCode = {eCode} | eValue = {eValue}")
            # if eCode == 1:
            #     self.setDirection(eValue)
            #     self.setSpeed(eValue)
            #     # time.sleep(0.1) # Delay for stability


"""
Handle signals and gracefully shutdown before exit
"""
def signal_handler(sgnl, _):
    # match sgnl:
    #     case signal.SIGINT:
    #         print("Caught SIGINT: Ctrl+C detected, cleaning up and shutting down")
    #     case signal.SIGTERM:
    #         print("Caught SIGTERM: cleaning up and shutting down")
    #     case signal.SIGKILL:
    #         print("Caught SIGKILL: cleaning up and shutting down")
    # print("Cleaning up GPIO")
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
    # parent_conn, child_conn = Pipe()
    ctlProcess = Process(target=ctlr.controller_input)
    # ctlProcess = Process(target=ctlr.read_controller)
    motorProcess = Process(target=motors.run)
    ctlProcess.start()
    procList.append(ctlProcess)
    motorProcess.start()
    procList.append(motorProcess)
    
    # Keep main process going
    try:
        while True:
            time.sleep(1)
    except Exception as e:
        print(repr(e))
        signal_handler(None, None)
    
    # signal.signal(signal.SIGKILL, signal_handler)
    # ctlProcess.join()
    # motorProcess.join()

if __name__ == '__main__':
    main()