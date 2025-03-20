/*!
 *@file detectGesture.ino
 *@brief Detect gestures
 *@details  This code detects the location, score of faces, and gestures along with their scores.
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
// Buffer for formatted output
char str[100];


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

    // Set the face detection threshold. Face scores range from 0 to 100.
    // Faces with scores above this threshold will be detected.
    gfd.setFaceDetectThres(60);

    // Set the gesture detection threshold. Gesture scores range from 0 to 100.
    // Gestures with scores above this threshold will be detected.
    gfd.setGestureDetectThres(60);    

    // Set the gesture detection range.
    // The range is from 0 to 100; 0 has the smallest detection range, and 100 has the largest.
    gfd.setDetectThres(100);
}


void loop(){
    // Check if any faces are detected
    if(gfd.getFaceNumber() > 0){

        // Retrieve face score and location
        uint16_t faceScore = gfd.getFaceScore();
        uint16_t faceX = gfd.getFaceLocationX();
        uint16_t faceY = gfd.getFaceLocationY();
        
        // Print the face detection results
        sprintf(str, "detect face at (x = %d, y = %d, score = %d)\n", faceX, faceY, faceScore);
        Serial.print(str);
        
        // Print the gesture detection results
        // - 1: LIKE (👍) - blue
        // - 2: OK (👌) - green
        // - 3: STOP (🤚) - red
        // - 4: YES (✌) - yellow
        // - 5: SIX (🤙) - purple
        uint16_t gestureType = gfd.getGestureType();
        uint16_t gestureScore = gfd.getGestureScore();
        
        // Print the gesture detection results
        sprintf(str, "detect gesture %d, score = %d\n", gestureType, gestureScore);
        Serial.print(str);
    }
    
    // Delay before the next loop iteration
    delay(500);
}
