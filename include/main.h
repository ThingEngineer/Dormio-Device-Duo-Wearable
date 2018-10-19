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
#include <WiFi.h>                      // ESP32 Wifi support
#include <HTTPClient.h>                // Library to easily make HTTP GET, POST and PUT requests to a web server
#include <WiFiClientSecure.h>          // Base class that provides Client SSL to ESP32
#include <MAX30105.h>                  // SparkFun MAX3010x Pulse and Proximity Sensor Library - https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library


/****************************************************************************
* Constructors
****************************************************************************/
MAX30105 hrSensor;


/****************************************************************************
* Function prototypes
****************************************************************************/
void getNextHeartRateSample();
void normalizeRedLED();
void doHttpPost();


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
int32_t redMean = 0;
int32_t irMean = 0;
byte redPulseAmplitude;                // Red LED current values (LED Pulse Amplitude)
byte redPulseAmplitudePrevious;        // Previous ^

#endif /* MAIN_H */
