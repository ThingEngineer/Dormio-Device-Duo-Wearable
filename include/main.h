/*
 * main.h
 *
 * Author: Josh Campbell
 */


#ifndef MAIN_H
#define MAIN_H

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 303
#error "AVR-GCC 3.3 or later is required, update AVR-GCC compiler!"
#endif


/****************************************************************************
* Includes
****************************************************************************/
#include "env.h"                       // Sensitive environment parameters
#include <Arduino.h>                   // Main include file for the Arduino SDK
#include <WiFi.h>                      // ESP32 Wifi support
#include <HTTPClient.h>                // Library to easily make HTTP GET, POST and PUT requests to a web server
#include <WiFiClientSecure.h>          // Base class that provides Client SSL to ESP32
#include <MAX30105.h>                  // SparkFun MAX3010x Pulse and Proximity Sensor Library - https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library


/****************************************************************************
* Constructors
****************************************************************************/
MAX30105 particleSensor;


/****************************************************************************
* Function prototypes
****************************************************************************/
void doHttpPost(void);


/****************************************************************************
* Port aliases
****************************************************************************/


/****************************************************************************
* Variable and constant declarations
****************************************************************************/


#endif /* MAIN_H */
