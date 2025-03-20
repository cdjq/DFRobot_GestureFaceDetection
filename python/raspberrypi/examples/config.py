'''
  @file config.py
  @brief Configure sensor parameters
  @details  This code configures the sensor's address and serial communication parameters.
  @copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT license (MIT)
  @author [thdyyl](yuanlong.yu@dfrobot.com)
  @version  V1.0
  @date  2025-03-17
  @https://github.com/DFRobot/DFRobot_GestureFaceDetection
'''

# -*- coding: utf-8 -*-
import time
import serial
import sys
sys.path.append("../")
from DFRobot_GestureFaceDetection import DFRobot_GestureFaceDetection_I2C, DFRobot_GestureFaceDetection_UART

# Define macros (simulated using variables)
USE_I2C = False  # Set to True to use I2C; set to False to use UART

# Define device address and baud rate
DEVICE_ID = 0x72
UART_BAUD_RATE = 9600

# Set a new device address
NEW_DEVICE_ID = 0x72
# Choose between I2C or UART based on the macro
if USE_I2C:
    # Using I2C interface
    gfd = DFRobot_GestureFaceDetection_I2C(bus=1, addr=DEVICE_ID)  # Assuming I2C bus 1 is used
else:
    # Using UART interface
    gfd = DFRobot_GestureFaceDetection_UART(baud=UART_BAUD_RATE, addr=DEVICE_ID)

def setup():
    """
    @brief Setup function for initializing the sensor and communication.
    
    This function initializes the sensor and configures the communication settings
    based on the selected interface (I2C or UART).
    """
    if USE_I2C:
        # I2C does not require additional initialization
        print("I2C setup complete.")
    else:
        # Configure UART communication
        gfd.config_uart(baud=gfd.EBAUD_115200, parity=gfd.UART_CFG_PARITY_NONE, stop_bit=gfd.UART_CFG_STOP_BITS_2)
        print("UART configured.")
    
    # Set the device address
    gfd.set_addr(NEW_DEVICE_ID)
    print("Device address set to: {}".format(NEW_DEVICE_ID))

def loop():
    """
    @brief Main loop function.
    
    This function runs in a loop, simulating a delay similar to Arduino's delay function.
    """
    while True:
        time.sleep(1)  # Simulate delay similar to Arduino's delay function

# Execute the setup function
setup()

# Execute the loop function
loop()
