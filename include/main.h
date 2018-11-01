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
#include <Arduino.h>                   // Main include file for the Arduino SDK
#include <Wire.h>                      // Arduino I2C library
#include <ESP8266WiFi.h>               // ESP8266 core for Arduino
#include <ESP8266HTTPClient.h>         // ESP8266 HTTP client
#include <MAX30105.h>                  // SparkFun MAX3010x Pulse and Proximity Sensor Library - https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include <Adafruit_MLX90614.h>         // MLX90614 temperature sensor library - https://github.com/adafruit/Adafruit-MLX90614-Library
#include <SparkFunLSM6DS3.h>           // LSM6DS3 accelerometer and gyroscope 6DoF IMU Library
// #include <Adafruit_DRV2605.h>          // DRV2605L Haptic Controller https://github.com/adafruit/Adafruit_DRV2605_Library
#include <Adafruit_SSD1306.h>          // SSD1306 OLED driver library for 'monochrome' 128x64 and 128x32 OLEDs - https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_GFX.h>              // Adafruit GFX Library

#include "env.h"                       // Secret values


/****************************************************************************
* Constructors
****************************************************************************/
MAX30105 hrSensor;
Adafruit_MLX90614 tempSensor = Adafruit_MLX90614();
LSM6DS3 IMU;
// Adafruit_DRV2605 hapticFeedback;
#define OLED_RESET 16
Adafruit_SSD1306 display(OLED_RESET);


/****************************************************************************
* Function prototypes
****************************************************************************/
void sampleRateFull();
void sampleRateModHalfSF();
void sampleRateSingle();
void httpPost();
void loadFloatBuffer(float _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset);
void load32Buffer(uint32_t _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset);
void load16Buffer(uint16_t _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset);
void loadMACBuffer(uint16_t _arrayOffset);


/****************************************************************************
* Port aliases
****************************************************************************/
const uint8_t GSRpin = A0;             // GSR analog input pin


/****************************************************************************
* Variable and constant declarations
****************************************************************************/
#define ST 5                           // Sampling time in seconds
#define SF 50                          // Sampling frequency in Hz - this should match MAX30102 (sampleRate / sampleAverage / 2) (2 because there is one sample for each, ir & red)
#define HALF_SF (SF / 2)               // Sampling frequency / 2
#define SAMPLE_COUNT (ST * SF)         // MAX30102 sample count per cycle

// Data array member sizes in totly bytes per cycle
#define IR_RED_SIZE (SAMPLE_COUNT * 4) // MAX30102 ir/red buffer - sample count * 4(bytes per sample)
#define DIE_TEMP_SIZE 3                // MAX30102 die temperture
#define OBJECT_TEMP_SIZE 3             // MLX90614 object temperture
#define AMBIENT_TEMP_SIZE 3            // MLX90614 ambient temperture
#define GSR_SIZE 20                    // LM324 GSR readings
#define ACCEL_X_SIZE 30                // LSM6DS3 accelerometer x readings
#define ACCEL_Y_SIZE 30                // LSM6DS3 accelerometer y readings
#define ACCEL_Z_SIZE 30                // LSM6DS3 accelerometer z readings
#define GYRO_X_SIZE 30                 // LSM6DS3 gyroscope x readings
#define GYRO_Y_SIZE 30                 // LSM6DS3 gyroscope y readings
#define GYRO_Z_SIZE 30                 // LSM6DS3 gyroscope z readings
#define EPOCH_SIZE 4                   // Epoch seconds
#define FRAME_COUNT_SIZE 2             // Frames captured
#define MAC_SIZE 6                     // MAC address
#define CHECKSUM_SIZE 1                // Checksum

// Data array offsets
#define IR_OFFSET 0                                                  // IR LED sensor
#define RED_OFFSET IR_RED_SIZE                                       // RED LED sensor
#define DIE_TEMP_OFFSET (RED_OFFSET + IR_RED_SIZE)                   // MAX30102 die temperture
#define SKIN_TEMP_OFFSET (DIE_TEMP_OFFSET + DIE_TEMP_SIZE)           // MLX90614 object (skin) temperture
#define AMBIENT_TEMP_OFFSET (SKIN_TEMP_OFFSET + OBJECT_TEMP_SIZE)    // MLX90614 ambient temperture
#define GSR_OFFSET (AMBIENT_TEMP_OFFSET + AMBIENT_TEMP_SIZE)         // LM324 GSR readings
#define ACCEL_X_OFFSET (GSR_OFFSET + GSR_SIZE)                       // LSM6DS3 accelerometer x readings
#define ACCEL_Y_OFFSET (ACCEL_X_OFFSET + ACCEL_X_SIZE)               // LSM6DS3 accelerometer y readings
#define ACCEL_Z_OFFSET (ACCEL_Y_OFFSET + ACCEL_Y_SIZE)               // LSM6DS3 accelerometer z readings
#define GYRO_X_OFFSET (ACCEL_Z_OFFSET + ACCEL_Z_SIZE)                // LSM6DS3 gyroscope x readings
#define GYRO_Y_OFFSET (GYRO_X_OFFSET + GYRO_X_SIZE)                  // LSM6DS3 gyroscope y readings
#define GYRO_Z_OFFSET (GYRO_Y_OFFSET + GYRO_Y_SIZE)                  // LSM6DS3 gyroscope z readings
#define EPOCH_OFFSET (GYRO_Z_OFFSET + GYRO_Z_SIZE)                   // Epoch seconds (time since boot)
#define FRAME_COUNT_OFFSET (EPOCH_OFFSET + EPOCH_SIZE)               // Frames captured
#define MAC_OFFSET (FRAME_COUNT_OFFSET + FRAME_COUNT_SIZE)           // MAC address
#define CHECKSUM_OFFSET (MAC_OFFSET + MAC_SIZE)                      // Checksum

#define BUFFER_SIZE (CHECKSUM_OFFSET + 1) // Total data buffer size
uint8_t dataBuffer[BUFFER_SIZE];       // Post data buffer - holds all sensor data to send
uint8_t mac[6];                        // MAC address of the ESP8266
uint8_t checksum;                      // Data array XOR checksum

uint8_t fullSampleCounter = 0;         // Full sample loop counter
uint8_t modSFSampleCounter = 0;        // Mod SF sample loop counter (SAMPLE_COUNT / SF)
uint16_t frameCounter = 0;             // Counts the number of sample frames captured


#endif /* MAIN_H */
