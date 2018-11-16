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
#include <Esp.h>                       // ESP8266-specific APIs
#include <FS.h>                        // ESP8266 file system wrapper
#include <ESP8266WiFi.h>               // ESP8266 core for Arduino
#include <ESP8266HTTPClient.h>         // HTTP client for ESP8266
#include <ESP8266httpUpdate.h>         // HTTP OTA Update for ESP8266
#include <WebSocketsClient.h>          // WebSocket Server and Client for Arduino
#include <DNSServer.h>                 // ESP8266 simple DNS server
#include <ESP8266WebServer.h>          // ESP8266 simple web-server
#include <WiFiManager.h>               // ESP8266 WiFi Connection manager with web captive portal - https://github.com/tzapu/WiFiManager
#include <JC_Button.h>                 // Arduino Button Library - https://github.com/JChristensen/JC_Button
#include <MAX30105.h>                  // SparkFun MAX3010x Pulse and Proximity Sensor Library - https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include <Adafruit_MLX90614.h>         // MLX90614 temperature sensor library - https://github.com/adafruit/Adafruit-MLX90614-Library
#include <SparkFunLSM6DS3.h>           // LSM6DS3 accelerometer and gyroscope 6DoF IMU Library
#include <Adafruit_DRV2605.h>          // DRV2605L Haptic Controller https://github.com/adafruit/Adafruit_DRV2605_Library
#include <Adafruit_SSD1306.h>          // SSD1306 OLED driver library for 'monochrome' 128x64 and 128x32 OLEDs - https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_GFX.h>              // Adafruit GFX Library
#include <Adafruit_ADS1015.h>          // ADS1115 high-resolution analog to digital converter with programmable gain amplifier

#include "env.h"                       // Secret values

/****************************************************************************
* Constructors
****************************************************************************/
WebSocketsClient webSocket;
Button *wifiBtn;
MAX30105 ppg;
Adafruit_MLX90614 irTherm = Adafruit_MLX90614();
LSM6DS3 imu;
Adafruit_DRV2605 haptic;
#define OLED_RESET 16
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_ADS1115 ads;

/****************************************************************************
* Function prototypes
****************************************************************************/
void connectToWiFi(boolean resetSettings = false);
void WiFiConfigMode(WiFiManager *myWiFiManager);
void WiFiSuccess();
void connectToWebSocket();
void wifiResetButton();
void sampleRateFull();
void sampleRateModHalfSF();
void sampleRateSingle();
void normalizePPG();
void httpPost();
void wsSendBuffer();
void encryptBuffer();
void loadFloatBuffer(float _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset);
void load32Buffer(uint32_t _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset);
void load16Buffer(uint16_t _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset);
void loadMACBuffer(uint16_t _arrayOffset);
void I2CSelect(uint8_t channel);
void calcChecksum(uint8_t newValue);
void checkForUpdates();
String getFormatedMAC();
void displayOn();
void displayOff();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);

/****************************************************************************
* Port aliases
****************************************************************************/
#define WIFI_BUTTON_PIN 0              // WiFi reset button on GPIO0 (D3)
#define GSRpin 0                       // Galvanic skin response ADS1115 analog input A0
#define ECGpin 1                       // ECG/EKG/EMG Electrocardiography/Electromyography ADS1115 analog input A1

/****************************************************************************
* Variable and constant declarations
****************************************************************************/
#define DEBUG_ESP_PORT Serial          // Comment to disable all serial debug output
#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

#define PULLUP true
#define INVERT true
#define DEBOUNCE_MS 20

#define ST 5                           // Sampling time in seconds
#define SF 50                          // Sampling frequency in Hz - this should match MAX30102 (sampleRate / sampleAverage / 2) (2 because there is one sample for each, ir & red)
#define HALF_SF (SF / 2)               // Sampling frequency / 2
#define SAMPLE_COUNT (ST * SF)         // MAX30102 sample count per frame

// Data array member sizes in totly bytes per frame
#define IR_RED_SIZE (SAMPLE_COUNT * 4) // MAX30102 ir/red buffer - sample count * 4(bytes per sample)
#define ECG_SIZE (SAMPLE_COUNT * 2)    // ECG ADC1115 adc sample - sample count * 2(bytes per sample)
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
#define VCC_SIZE 2                     // VCC adc reading
#define CHECKSUM_SIZE 1                // Checksum

// Data array offsets
#define IR_OFFSET 0                                                  // PPG IR LED readings
#define RED_OFFSET IR_RED_SIZE                                       // PPG RED LED readings
#define ECG_OFFSET (RED_OFFSET + IR_RED_SIZE)                        // ECG readings
#define DIE_TEMP_OFFSET (ECG_OFFSET + ECG_SIZE)                      // MAX30102 die temperture
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
#define VCC_OFFSET (MAC_OFFSET + MAC_SIZE)                           // VCC adc reading
#define CHECKSUM_OFFSET (VCC_OFFSET + VCC_SIZE)                      // Checksum

#define BUFFER_SIZE (CHECKSUM_OFFSET + 1) // Total data buffer size
uint8_t redPulseAmplitude;             // Holds the variable red LED amplidude value
uint8_t redPulseAmplitudePrevious;     // Previous iteration value for the red LED amplidude
int32_t irBuffer[SF], redBuffer[SF];   // Raw IR/Red PPG signal used to normalize Red LED
uint8_t bufferCounter = 0;             // Data buffer index counter for the above IR/Red data
uint8_t dataBuffer[BUFFER_SIZE];       // Post data buffer - holds all sensor data to send
uint8_t mac[6];                        // MAC address of the ESP8266
uint8_t checksum = 0;                  // Data array XOR checksum
uint8_t fullSampleCounter = 0;         // Full sample loop counter
uint8_t modSFSampleCounter = 0;        // Mod SF sample loop counter (SAMPLE_COUNT / SF)
uint16_t frameCounter = 0;             // Counts the number of sample frames captured

unsigned long pressedAtMillis;         // Button press start timer
unsigned long const longPressInterval = 3000; // Milliseconds for button press to count as a long press
unsigned long pressedForMillis;        // Button press durration timer
boolean displayStatus = true;          // Display status, true = on, false = off

bool isConnected = false;

ADC_MODE(ADC_VCC);

#endif /* MAIN_H */
