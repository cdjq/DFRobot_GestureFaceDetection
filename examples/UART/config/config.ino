/*!
 *@file config.ino
 *@brief Configure sensor parameters
 *@details  This code configures the sensor's address and serial communication parameters.
 *@copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2025-03-17
 *@https://github.com/DFRobot/DFRobot_GestureFaceDetection
*/

#include "DFRobot_GestureFaceDetection.h"

// Define the device ID for the GestureFaceDetection sensor
#define DEVICE_ID  0x72 
#define DEVICE_BAUD 9600

 /* ---------------------------------------------------------------------------------------------------------------------
  *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
  *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
  *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
  *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
  *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
  * ----------------------------------------------------------------------------------------------------------------------*/
  /* Baud rate cannot be changed  */
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  #include <SoftwareSerial.h>
  SoftwareSerial mySerial1(4, 5);
  DFRobot_GestureFaceDetection_UART gfd(&mySerial1, DEVICE_ID);
#else
  // Create an instance of DFRobot_GestureFaceDetection_UART with the specified device ID and Serial1 for UART communicatio
  DFRobot_GestureFaceDetection_UART gfd(&Serial1, DEVICE_ID);
#endif




void setup(){

    // Initialize Serial1 for UART communication with the sensor
    #if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
      mySerial1.begin(DEVICE_BAUD);
    #elif defined(ESP32)
      Serial1.begin(DEVICE_BAUD, SERIAL_8N1, D2, D3);
    #else
      Serial1.begin(DEVICE_BAUD);
    #endif
    // Initialize serial communication for debugging purposes 
    Serial.begin(115200);

    // Configure the UART settings of the sensor
    gfd.configUart(eBaud_115200, UART_CFG_PARITY_NONE, UART_CFG_STOP_BITS_2);

    // Set the device address of the sensor
    gfd.setDeviceAddr(0x77);
}


void loop()
{
    // Main loop code would go here
}
