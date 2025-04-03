# -*- coding: utf-8 -*-
'''
  @file DFRobot_GestureFaceDetection.py
  @brief Define the basic structure and methods of the DFRobot_GestureFaceDetection class.
  @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT license (MIT)
  @author [thdyyl](yuanlong.yu@dfrobot.com)
  @version  V1.0
  @date  2025-03-17
  @https://github.com/DFRobot/DFRobot_GestureFaceDetection
'''

import serial
import time
from DFRobot_RTU import *
from smbus2 import SMBus, i2c_msg

class DFRobot_GestureFaceDetection(object):
    # Define register address constants
    REG_GFD_ADDR = 0x0000                       #< Device address register
    REG_GFD_BAUDRATE = 0x0001                   #< Baud rate configuration register
    REG_GFD_VERIFY_AND_STOP = 0x0002            #< Parity and stop bits configuration register
    REG_GFD_FACE_THRESHOLD = 0x0003             #< Face detection threshold, X coordinate
    REG_GFD_FACE_SCORE_THRESHOLD = 0x0004       #< Face score threshold
    REG_GFD_GESTURE_SCORE_THRESHOLD = 0x0005    #< Gesture score threshold

    GFD_PID = 0x0272                            #< Product ID

    REG_GFD_PID = 0x0000                        #< Product ID register
    REG_GFD_VID = 0x0001                        #< Vendor ID register
    REG_GFD_HW_VERSION = 0x0002                 #< Hardware version register
    REG_GFD_SW_VERSION = 0x0003                 #< Software version register
    REG_GFD_FACE_NUMBER = 0x0004                #< Number of detected faces
    REG_GFD_FACE_LOCATION_X = 0x0005            #< Face X coordinate
    REG_GFD_FACE_LOCATION_Y = 0x0006            #< Face Y coordinate
    REG_GFD_FACE_SCORE = 0x0007                 #< Face score
    REG_GFD_GESTURE_TYPE = 0x0008               #< Gesture type
    REG_GFD_GESTURE_SCORE = 0x0009              #< Gesture score

    INPUT_REG_OFFSET = 0x06                     #< Input register offset


    EBAUD_1200 = 1         #< Baud rate 1200
    EBAUD_2400 = 2         #< Baud rate 2400
    EBAUD_4800 = 3         #< Baud rate 4800
    EBAUD_9600 = 4         #< Baud rate 9600
    EBAUD_14400 = 5        #< Baud rate 14400
    EBAUD_19200 = 6        #< Baud rate 19200
    EBAUD_38400 = 7        #< Baud rate 38400
    EBAUD_57600 = 8        #< Baud rate 57600
    EBAUD_115200 = 9       #< Baud rate 115200
    EBAUD_230400 = 10      #< Baud rate 230400
    EBAUD_460800 = 11      #< Baud rate 460800
    EBAUD_921600 = 12      #< Baud rate 921600

    UART_CFG_PARITY_NONE = 0      #< No parity
    UART_CFG_PARITY_ODD = 1       #< Odd parity
    UART_CFG_PARITY_EVEN = 2      #< Even parity
    UART_CFG_PARITY_MARK = 3      #< Mark parity
    UART_CFG_PARITY_SPACE = 4     #< Space parity

    UART_CFG_STOP_BITS_0_5 = 0    #< 0.5 stop bits
    UART_CFG_STOP_BITS_1 = 1      #< 1 stop bit
    UART_CFG_STOP_BITS_1_5 = 2    #< 1.5 stop bits
    UART_CFG_STOP_BITS_2 = 3      #< 2 stop bits

    I2C_RETRY_MAX = 3
    def __init__(self):
        # Initialize the class
        pass

    '''
      @brief Init function
      @return True if initialization is successful, otherwise false.
    '''
    def begin(self):
        if self.readInputReg(self.REG_GFD_PID) == self.GFD_PID:
            return True
        return False

    '''
      @brief Get the device PID
      @return Returns the device PID
    '''
    def read_pid(self):
        return self.readInputReg(self.REG_GFD_PID)

    '''
      @brief Get the device VID
      @return Returns the device VID
    '''
    def read_vid(self):
        return self.readInputReg(self.REG_GFD_VID)

    '''
      @brief Get the number of detected faces
      @return Returns the number of detected faces
    '''
    def get_face_number(self):
        return self.readInputReg(self.REG_GFD_FACE_NUMBER)

    '''
      @brief Get the X location of the face
      @return Returns the X location
    '''
    def get_face_location_x(self):
        return self.readInputReg(self.REG_GFD_FACE_LOCATION_X)

    '''
      @brief Get the Y location of the face
      @return Returns the Y location
    '''
    def get_face_location_y(self):
        return self.readInputReg(self.REG_GFD_FACE_LOCATION_Y)

    '''
      @brief Get the face score
      @return Returns the face score
    '''
    def get_face_score(self):
        return self.readInputReg(self.REG_GFD_FACE_SCORE)

    '''
      @brief Get the gesture type
             - 1: LIKE (👍) - blue
             - 2: OK (👌) - green
             - 3: STOP (🤚) - red
             - 4: YES (✌) - yellow
             - 5: SIX (🤙) - purple
      @return Returns the gesture type
    '''
    def get_gesture_type(self):
        return self.readInputReg(self.REG_GFD_GESTURE_TYPE)

    '''
      @brief Get the gesture score
      @return Returns the gesture score
    '''
    def get_gesture_score(self):
        return self.readInputReg(self.REG_GFD_GESTURE_SCORE)

    '''
      @brief Set the face detection threshold
      @n Sets the threshold for face detection (0-100). Default is 60%
      @param score Threshold score
    '''
    def set_face_detect_thres(self, score):
        if (0 >= score) or (score > 100):
            return False
        return self.writeHoldingReg(self.REG_GFD_FACE_SCORE_THRESHOLD, score)

    '''
      @brief Get the face detection threshold
      @n Get the threshold for face detection (0-100). Default is 60%
    '''
    def get_face_detect_thres(self):
        return self.readHoldingReg(self.REG_GFD_FACE_SCORE_THRESHOLD)

    '''
      @brief Set the face score threshold
      @n Sets the threshold for detecting the X coordinate (0-100). Default is 60%.
      @param x Threshold value
    '''
    def set_detect_thres(self, x):
        if (0 >= x) or (x > 100):
            return False
        return self.writeHoldingReg(self.REG_GFD_FACE_THRESHOLD, x)

    '''
      @brief Get the face score threshold
      @n Get the threshold for detecting the X coordinate (0-100). Default is 60%.
    '''
    def get_detect_thres(self):
        return self.readHoldingReg(self.REG_GFD_FACE_THRESHOLD)

    '''
      @brief Set the gesture detection threshold
      @n Sets the threshold for gesture detection (0-100). Default is 60%.
      @param score Threshold score
    '''
    def set_gesture_detect_thres(self, score):
        if (0 >= score) or (score > 100):
            return False
        return self.writeHoldingReg(self.REG_GFD_GESTURE_SCORE_THRESHOLD, score)

    '''
      @brief Get the gesture detection threshold
      @n Get the threshold for gesture detection (0-100). Default is 60%.
    '''
    def get_gesture_detect_thres(self):
        return self.readHoldingReg(self.REG_GFD_GESTURE_SCORE_THRESHOLD)

    '''
      @brief Set the device address
      @param addr Address to set
    '''
    def set_addr(self, addr):
        if (addr < 1) or (addr > 0xF7):
            return False
        return self.writeHoldingReg(self.REG_GFD_ADDR, addr)

class DFRobot_GestureFaceDetection_I2C(DFRobot_GestureFaceDetection): 
    def __init__(self, bus, addr):
        # Initialize I2C address and bus
        self.__addr = addr
        self.__i2cbus = bus
        super(DFRobot_GestureFaceDetection_I2C, self).__init__()

    def calculate_crc(self, data):
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
            crc &= 0xFF
        return crc

    '''
      @fn write_reg
      @brief Write data to a register
      @param reg 16-bit register address
      @param data 8-bit register value
    '''
    def write_reg(self, reg, data):
        # Split data into high and low 8 bits and write to I2C register
        val_high_byte = (data >> 8) & 0xFF 
        val_low_byte = data & 0xFF    
        reg_high_byte = (reg >> 8) & 0xFF
        reg_low_byte = reg & 0xFF
        crc = self.calculate_crc([reg_high_byte, reg_low_byte, val_high_byte, val_low_byte])
        for i in range(self.I2C_RETRY_MAX):
            with SMBus(self.__i2cbus) as bus:
                msg = i2c_msg.write(self.__addr, [reg_high_byte, reg_low_byte, val_high_byte, val_low_byte, crc])
                bus.i2c_rdwr(msg)
                time.sleep(0.05) # Because the slave has a clock extension, it needs to wait.
                msg = i2c_msg.read(self.__addr, 3)
                bus.i2c_rdwr(msg)
                data = list(msg)
                ret_data = (data[0] << 8) | data[1]
                if self.calculate_crc(data[:2]) == data[2] and ret_data == crc:
                    return True
        return False
    '''
      @fn read_reg
      @brief Read data from a register
      @param reg 16-bit register address
      @param length Length of data to read
      @return Data read from the register
    '''
    def read_reg(self, reg, length):
        reg_high_byte = (reg >> 8) & 0xFF 
        reg_low_byte = reg & 0xFF         
        crc = self.calculate_crc([reg_high_byte, reg_low_byte])
        for i in range(self.I2C_RETRY_MAX):
            with SMBus(self.__i2cbus) as bus:
                msg = i2c_msg.write(self.__addr, [reg_high_byte, reg_low_byte, crc])
                bus.i2c_rdwr(msg)
                time.sleep(0.02)
                msg = i2c_msg.read(self.__addr, 3)
                bus.i2c_rdwr(msg)
                data = list(msg)
                ret_data = (data[0] << 8) | data[1]
                if self.calculate_crc(data[:length]) == data[length] and ret_data != 0xFFFF:
                    return ret_data
        return 0

    def writeHoldingReg(self, reg, data):
        return self.write_reg(reg, data)

    def readInputReg(self, reg):
        return self.read_reg(self.INPUT_REG_OFFSET + reg, 2)

    def readHoldingReg(self, reg):
        return self.read_reg(reg, 2)

class DFRobot_GestureFaceDetection_UART(DFRobot_GestureFaceDetection, DFRobot_RTU): 
    def __init__(self, baud, addr):
        # Initialize UART baud rate and address
        self.__baud = baud
        self.__addr = addr
        DFRobot_GestureFaceDetection.__init__(self)
        DFRobot_RTU.__init__(self, baud, 8, 'N', 1)

    '''
      @brief Configure UART
      @param baud Baud rate
      @param parity Parity bit
      @param stop_bit Stop bits
    '''
    def config_uart(self, baud, parity, stop_bit):
        # Combine parity and stop bits into a 16-bit value
        verify_and_stop = (parity << 8) | (stop_bit & 0xff)
        # Set baud rate
        self.writeHoldingReg(self.REG_GFD_BAUDRATE, baud)
        # Set parity and stop bits
        return self.writeHoldingReg(self.REG_GFD_VERIFY_AND_STOP, verify_and_stop)

    def writeHoldingReg(self, reg, data):
        ret = self.write_holding_register(self.__addr, reg, data)
        return ret == 0

    def readInputReg(self, reg):
        try:
            data = self.read_input_registers(self.__addr, reg, 1)
            
            # Ensure data list has at least three elements
            if len(data) >= 3:
                regData = (data[1] << 8) | data[2]
            else:
                regData = 0
            
            return regData
        
        except Exception as e:
            return 0
    
    def readHoldingReg(self, reg):
        try:
            data = self.read_holding_registers(self.__addr, reg, 1)
            
            # Ensure data list has at least three elements
            if len(data) >= 3:
                regData = (data[1] << 8) | data[2]
            else:
                regData = 0
            
            return regData
        except Exception as e:
            return 0
