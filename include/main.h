/*
 * main.h
 *
 * Author: Josh Campbell
 */


#ifndef MAIN_H
#define MAIN_H


/****************************************************************************
* Includes
****************************************************************************/
#include "env.h"                       // Sensitive environment parameters
#include <Arduino.h>                   // Main include file for the Arduino SDK
#include <Wire.h>                      // Arduino I2C library
#include <ESP8266WiFi.h>               // ESP8266 core for Arduino
#include <ESP8266HTTPClient.h>         // ESP8266 HTTP client
#include <MAX30105.h>                  // SparkFun MAX3010x Pulse and Proximity Sensor Library - https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include <Adafruit_MLX90614.h>         // MLX90614 temperature sensor library - https://github.com/adafruit/Adafruit-MLX90614-Library
#include <SparkFunLSM6DS3.h>           // LSM6DS3 accelerometer and gyroscope 6DoF IMU Library - https://github.com/ThingEngineer/SparkFun_LSM6DS3_Arduino_Library
#include <Adafruit_DRV2605.h>          // DRV2605L Haptic Controller https://github.com/adafruit/Adafruit_DRV2605_Library
#include <Adafruit_SSD1306.h>          // SSD1306 OLED driver library for 'monochrome' 128x64 and 128x32 OLEDs - https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_GFX.h>              // Adafruit GFX Library


/****************************************************************************
* Constructors
****************************************************************************/
MAX30105 hrSensor;
Adafruit_MLX90614 tempSensor = Adafruit_MLX90614();
LSM6DS3 IMU;
Adafruit_DRV2605 hapticFeedback;
#define OLED_RESET 16
Adafruit_SSD1306 display(OLED_RESET);

/****************************************************************************
* Function prototypes
****************************************************************************/
void getNextHeartRateSample();
void normalizeRedLED();
void httpPost();


/****************************************************************************
* Port aliases
****************************************************************************/


/****************************************************************************
* Variable and constant declarations
****************************************************************************/
#define ST 5                           // Sampling time in seconds
#define SF 50                          // Sampling frequency in Hz - this should match MAX30102 (sampleRate / sampleAverage / 2) (2 because there is one sample for each, ir & red)
const byte BUFFER_SIZE = (ST * SF);    // MAX30102 buffer size

byte hrBufferCounter = 0;              // Data points captured/counted
int32_t irBuffer[BUFFER_SIZE];         // Infrared LED sensor data
int32_t redBuffer[BUFFER_SIZE];        // Red LED sensor data

#endif /* MAIN_H */
