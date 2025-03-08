import RPi.GPIO as GPIO
from evdev import InputDevice, categorize, ecodes
from time import sleep
import asyncio

BL_MOTOR_PIN_A = 17 # Pin 11
BL_MOTOR_PIN_B = 22 # Pin 15
FL_MOTOR_PIN_A = 24 # Pin 18
FL_MOTOR_PIN_B = 23 # Pin 16

# Find the device path. You might need to check /dev/input/event* for the correct one.
controller = InputDevice('/dev/input/event4') # Replace X with the correct event number

print(f"Successfully connected to {controller.name}")
print(controller.phys)

sleep(2)

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
async def read_controller():
    async for event in controller.async_read_loop():
        if event.type == ecodes.EV_ABS:
            print(f"type = {event.type} | code = {event.code} | value = {event.value}")

asyncio.run(read_controller())
