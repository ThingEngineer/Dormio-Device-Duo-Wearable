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
#include <SparkFunLSM6DS3.h>           // LSM6DS3 accelerometer and gyroscope 6DoF IMU Library
// #include <Adafruit_DRV2605.h>          // DRV2605L Haptic Controller https://github.com/adafruit/Adafruit_DRV2605_Library
#include <Adafruit_SSD1306.h>          // SSD1306 OLED driver library for 'monochrome' 128x64 and 128x32 OLEDs - https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_GFX.h>              // Adafruit GFX Library


/****************************************************************************
* Constructors
****************************************************************************/
MAX30105 hrSensor;
Adafruit_MLX90614 tempSensor = Adafruit_MLX90614();
LSM6DS3 IMU;
// Adafruit_DRV2605 hapticFeedback;
#define OLED_RESET 16
Adafruit_SSD1306 display(OLED_RESET);
movingAvgFloat accelXAvg(4);
movingAvgFloat accelYAvg(4);
movingAvgFloat accelZAvg(4);
movingAvgFloat gyroXAvg(4);
movingAvgFloat gyroYAvg(4);
movingAvgFloat gyroZAvg(4);


/****************************************************************************
* Function prototypes
****************************************************************************/
void sampleRateFull();
void sampleRateModSF();
void sampleRateSingle();
void httpPost();


/****************************************************************************
* Port aliases
****************************************************************************/


/****************************************************************************
* Variable and constant declarations
****************************************************************************/
#define ST 5                           // Sampling time in seconds
#define SF 50                          // Sampling frequency in Hz - this should match MAX30102 (sampleRate / sampleAverage / 2) (2 because there is one sample for each, ir & red)
#define SAMPLE_COUNT (ST * SF)         // MAX30102 sample count per cycle

// Data array member sizes
#define IR_RED_SIZE (SAMPLE_COUNT * 4) // MAX30102 ir/red buffer - sample count * 4(bytes per sample)
#define DIE_TEMP_SIZE 4                // MAX30102 die temperture
#define OBJECT_TEMP_SIZE 4             // MLX90614 object temperture
#define AMBIENT_TEMP_SIZE 4            // MLX90614 ambient temperture
#define GSR_SIZE 10                    // LM324 GSR readings
#define ACCEL_X_SIZE 20                // LSM6DS3 accelerometer x readings
#define ACCEL_Y_SIZE 20                // LSM6DS3 accelerometer y readings
#define ACCEL_Z_SIZE 20                // LSM6DS3 accelerometer z readings
#define GYRO_X_SIZE 20                 // LSM6DS3 gyroscope x readings
#define GYRO_Y_SIZE 20                 // LSM6DS3 gyroscope y readings
#define GYRO_Z_SIZE 20                 // LSM6DS3 gyroscope z readings
#define EPOCH_SIZE 4                   // Epoch seconds
#define CHECKSUM_SIZE 1                // Checksum

// Data array offsets
#define IR_OFFSET 0                                                  // IR LED sensor
#define RED_OFFSET IR_RED_SIZE                                       // RED LED sensor
#define DIE_TEMP_OFFSET (RED_OFFSET + IR_RED_SIZE)                   // MAX30102 die temperture
#define OBJECT_TEMP_OFFSET (DIE_TEMP_OFFSET + DIE_TEMP_SIZE)         // MLX90614 object temperture
#define AMBIENT_TEMP_OFFSET (OBJECT_TEMP_OFFSET + OBJECT_TEMP_SIZE)  // MLX90614 ambient temperture
#define GSR_OFFSET (AMBIENT_TEMP_OFFSET + AMBIENT_TEMP_SIZE)         // LM324 GSR readings
#define ACCEL_X_OFFSET (GSR_OFFSET + GSR_SIZE)                       // LSM6DS3 accelerometer x readings
#define ACCEL_Y_OFFSET (ACCEL_X_OFFSET + ACCEL_X_SIZE)               // LSM6DS3 accelerometer y readings
#define ACCEL_Z_OFFSET (ACCEL_Y_OFFSET + ACCEL_Y_SIZE)               // LSM6DS3 accelerometer z readings
#define GYRO_X_OFFSET (ACCEL_Z_OFFSET + ACCEL_Z_SIZE)                // LSM6DS3 gyroscope x readings
#define GYRO_Y_OFFSET (GYRO_X_OFFSET + GYRO_X_SIZE)                  // LSM6DS3 gyroscope y readings
#define GYRO_Z_OFFSET (GYRO_Y_OFFSET + GYRO_Y_SIZE)                  // LSM6DS3 gyroscope z readings
#define EPOCH_OFFSET (GYRO_Z_OFFSET + GYRO_Z_SIZE)                   // Epoch seconds
#define CHECKSUM_OFFSET (EPOCH_OFFSET + EPOCH_SIZE)                  // Checksum

#define BUFFER_SIZE (CHECKSUM_OFFSET)     // Total data buffer size
uint8_t dataBuffer[BUFFER_SIZE];          // Post data buffer - holds all sensor data to send
uint8_t checksum;                         // Data array XOR checksum

uint8_t hrSampleCounter = 0;              // MAX30102 sample loop counter

const uint8_t GSRpin = A0;                // GSR analog input pin


#endif /* MAIN_H */
