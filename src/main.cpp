#include "main.h"

/**
 * Initialization routines
 */
void setup() {
  #ifdef DEBUG_ESP_PORT
  DEBUG_ESP_PORT.begin(921600);        // Initialize serial data transmission and set baud rate
  DEBUG_ESP_PORT.setDebugOutput(true); // Enable ESP8266 debug output
  #endif

  wifiBtn = new Button(WIFI_BUTTON_PIN, PULLUP, INVERT, DEBOUNCE_MS);

  pinMode(BEEPER, OUTPUT);             // Initilize beeper pin
  digitalWrite(BEEPER, HIGH);          // Short beep
  delay(100);
  digitalWrite(BEEPER, LOW);

  /************************************I2C************************************/
  Wire.begin(4,5);                     // Initial I2C and join bus as master

  I2CSelect(6);                        // Select I2C bus channel 6
  haptic.begin();                      // Initialize DRV2605 haptic feedback driver
  haptic.setMode(DRV2605_MODE_INTTRIG);// Set DRV2605 trigger mode
  haptic.selectLibrary(1);             // Set haptic feedback library
  haptic.setWaveform(1, 1);            // Strong click 100%, see datasheet part 11.2

  I2CSelect(7);                        // Select I2C bus channel 7
  irTherm.begin();                     // Initilize MLX90614 temperature sensor
  imu.begin();                         // Initialize LSM6DS3 6DOF IMU
  ads.begin();                         // Initialize ADS1115 ADC
  ads.setGain(GAIN_ONE);               // Default ADS1115 amplifier gain

  uint8_t ledBrightness = 32;          // Options: 0=Off to 255=50mA
  uint8_t sampleAverage = 8;           // Options: 1, 2, 4, 8, 16, 32
  uint8_t ledMode = 2;                 // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  uint16_t sampleRate = 800;           // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  uint16_t pulseWidth = 215;           // Options: 69, 118, 215, 411
  uint16_t adcRange = 2048;            // Options: 2048, 4096, 8192, 16384
  redPulseAmplitude = ledBrightness;   // Initial variable red LED pulse amplitude
  ppg.begin();                         // Initialize MAX30102
  ppg.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Set MAX30102 options
  ppg.readTemperature();               // Initialize die temperture reading

  /************************************OLED***********************************/
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  /***********************************ESP8266*********************************/
  display.setCursor(0,0);
  display.println(F("Dormio Device Duo"));
  display.println(F("--==Deep Dream==--"));
  display.println(F("Connecting to WiFi"));
  display.display();
  delay(1000);

  connectToWiFi();
  WiFiSuccess();
  delay(2000);
  checkForUpdates();
  delay(2000);
  connectToWebSocket();
}

/**
 * Main program loop
 */
void loop() {
  wifiResetButton(); // Check for wifi reset button press

  webSocket.loop();

  if(isConnected) {

    sampleRateFull(); // Full sample rate (50 per second)

    // if ( bufferCounter == SF ) normalizePPG(); // Every SF interval (50 samples)
    if ( bufferCounter == SF ) bufferCounter = 0;

    if ( fullSampleCounter % HALF_SF == 0) sampleRateModHalfSF(); // SAMPLE_COUNT/HALF_SF (2 per second)

    if ( fullSampleCounter == SAMPLE_COUNT ) {
      sampleRateSingle(); // Minimum sample rate (1 every 5 seconds)
      encryptBuffer(); // Encrypt the data buffer before sending
      // httpPost(); // Sample frame complete, send data over http
      wsSendBuffer(); // Sample frame complete, send data over WebSockets
      bufferCounter = 0;
      fullSampleCounter = 0; // Frame complete, reset counters
      modSFSampleCounter = 0;
      checksum = 0; // Reset checksum
    }

  }
}

/**
 * Function holding routines excecuted at the maximum sampling frequency per frame
 */
void sampleRateFull() {
  // Load IMU redings to data buffers
  irBuffer[bufferCounter] = ppg.getIR();
  redBuffer[bufferCounter] = ppg.getRed();
  load32Buffer(irBuffer[bufferCounter], fullSampleCounter, IR_OFFSET);
  load32Buffer(redBuffer[bufferCounter], fullSampleCounter, RED_OFFSET);
  // Load ECG reading to data buffer
  load16Buffer(ads.readADC_SingleEnded(ECGpin), fullSampleCounter, ECG_OFFSET);

  ppg.nextSample();
  fullSampleCounter++;
  bufferCounter++;
}

/**
 * Function holding routines excecuted at multiples of SAMPLE_COUNT/HALF_SF per frame
 */
void sampleRateModHalfSF() {
  // Load IMU redings to data buffer
  loadFloatBuffer(imu.readFloatAccelX(), modSFSampleCounter, ACCEL_X_OFFSET);
  loadFloatBuffer(imu.readFloatAccelY(), modSFSampleCounter, ACCEL_Y_OFFSET);
  loadFloatBuffer(imu.readFloatAccelZ(), modSFSampleCounter, ACCEL_Z_OFFSET);
  loadFloatBuffer(imu.readFloatGyroX(), modSFSampleCounter, GYRO_X_OFFSET);
  loadFloatBuffer(imu.readFloatGyroY(), modSFSampleCounter, GYRO_Y_OFFSET);
  loadFloatBuffer(imu.readFloatGyroZ(), modSFSampleCounter, GYRO_Z_OFFSET);
  // Load GSR reading to data buffer
  load16Buffer(ads.readADC_SingleEnded(GSRpin), modSFSampleCounter, GSR_OFFSET);

  modSFSampleCounter++;
}

/**
 * Function holding routines excecuted once per frame
 */
void sampleRateSingle() {
  loadFloatBuffer(ppg.readTemperature(), 0, DIE_TEMP_OFFSET); // Load current MAX30102 die temperture to data buffer
  loadFloatBuffer(irTherm.readObjectTempC(), 0, SKIN_TEMP_OFFSET); // Load current skin temperture to data buffer
  loadFloatBuffer(irTherm.readAmbientTempC(), 0, AMBIENT_TEMP_OFFSET); // Load current ambient temperture to data buffer
  load32Buffer((millis() / 1000), 0, EPOCH_OFFSET); // Load current epoch to data buffer
  load32Buffer(frameCounter, 0, FRAME_COUNT_OFFSET); // Load current frame count to data buffer
  loadMACBuffer(MAC_OFFSET); // Load MAC address into data buffer
  load16Buffer(ESP.getVcc(), 0, VCC_OFFSET);
  dataBuffer[CHECKSUM_OFFSET] = checksum; // Load checksum to data buffer

  display.clearDisplay();
  display.setCursor(0,0);
  // display.println((millis() / 1000));
  display.println(frameCounter);
  display.println(dataBuffer[CHECKSUM_OFFSET]);
  display.display();

  frameCounter++;
}

/**
 * Normalize IR and Red signals to within +/-8000
 * by adjusting the pulse amplidude of the Red LED only
 *
 * Each redPulseAmplitude step results in a change of about 8000
 * which means this is the minimum range without causing excessive jitter
 */
void normalizePPG() {
  // Calculate Red & IR DC mean
  int32_t redMean = 0;
  int32_t irMean = 0;
  int _bufferCounter = 0;
  for (_bufferCounter = 0 ; _bufferCounter < SF ; _bufferCounter++)
  {
    redMean += redBuffer[_bufferCounter];
    irMean += irBuffer[_bufferCounter];
  }
  redMean = redMean/SF;
  irMean = irMean/SF;

  // Adjust red LED current if it is to far above or below IR LED reading
  if ( irMean > (redMean + 8000) ) redPulseAmplitude++;
  if ( irMean < (redMean - 8000) ) redPulseAmplitude--;

  if ( redPulseAmplitudePrevious != redPulseAmplitude )
  {
    ppg.setPulseAmplitudeRed(redPulseAmplitude);   // Set new red LED current

    display.clearDisplay();
    display.setCursor(0,9);
    display.println(F("Normalizing PPG LED"));
    display.print(F("Amplidude: "));
    display.println(redPulseAmplitude);
    display.display();
  }
  redPulseAmplitudePrevious = redPulseAmplitude;
  bufferCounter = 0;
}

/**
 * Load sensor data into data buffer array - 32 bit floating point converted to 8 bit array[3], (uint16_t).(uint8_t)
 *
 * @param _bufferTemp    Value to be loaded into the data buffer
 * @param _sampleCounter Sample loop counter
 * @param _arrayOffset   Data array offset
 */
void loadFloatBuffer(float _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset) {
  int16_t characteristic = (int16_t)(_bufferTemp); // Remove the mantissa retaining just a signed integer portion of the value
  if (characteristic < 0.0) characteristic = (characteristic ^ 0x7FFF) + 1; // If negative remove two's complement signing, leave only the significant bit
  // Load the characteristic of the floating point to the data buffer
  uint16_t _byteOffset;
  uint8_t _shiftOffset;
  for ( int i = 0; i < 2; i++ ) // Load sensor data into data buffer array - 16 bit Little endian to 8 bit array[2]
  {
    _byteOffset = ((_sampleCounter * 3) + i); // Offset 3 bytes to account for the matissa as well
    _shiftOffset = (8 * i);

    dataBuffer[_arrayOffset + _byteOffset] = ( characteristic >> _shiftOffset );
    calcChecksum(dataBuffer[_arrayOffset + _byteOffset]); // Add new buffer value to checksum
  }

  float _mantissa = _bufferTemp - (int32_t)(_bufferTemp); // Remove the characteristic from the mantissa
  if (_mantissa < 0.0) _mantissa = _mantissa * -1.0; // abs of float
  uint8_t mantissa = (uint8_t)100 * _mantissa; // Retain 2 decimal places of the mantissa as a whole number
  _byteOffset = (_sampleCounter * 3); // Offset 3 bytes to account for the characteristic as well
  dataBuffer[_arrayOffset + _byteOffset + 2] = mantissa; // Load the mantissa of the floating point to the data buffer
  calcChecksum(mantissa); // Add new buffer value to checksum
}

/**
 * Load sensor data into data buffer array - 32 bit Little endian to 8 bit array[4]
 *
 * @param _bufferTemp    Value to be loaded into the data buffer
 * @param _sampleCounter Sample loop counter
 * @param _arrayOffset   Data array offset
 */
void load32Buffer(uint32_t _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset) {
  uint16_t _byteOffset;
  uint8_t _shiftOffset;
  for ( int i = 0; i < 4; i++ )
  {
    _byteOffset = ((_sampleCounter * 4) + i);
    _shiftOffset = (8 * i);

    dataBuffer[_arrayOffset + _byteOffset] = ( _bufferTemp >> _shiftOffset );
    calcChecksum(dataBuffer[_arrayOffset + _byteOffset]); // Add new buffer value to checksum
  }
}

/**
 * Load sensor data into data buffer array - 16 bit Little endian to 8 bit array[2]
 *
 * @param _bufferTemp    Value to be loaded into the data buffer
 * @param _sampleCounter Sample loop counter
 * @param _arrayOffset   Data array offset
 */
void load16Buffer(uint16_t _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset) {
  uint16_t _byteOffset;
  uint8_t _shiftOffset;
  for ( int i = 0; i < 2; i++ )
  {
    _byteOffset = ((_sampleCounter * 2) + i);
    _shiftOffset = (8 * i);

    dataBuffer[_arrayOffset + _byteOffset] = ( _bufferTemp >> _shiftOffset );
    calcChecksum(dataBuffer[_arrayOffset + _byteOffset]); // Add new buffer value to checksum
  }
}

/**
 * Load MAC address array into the data buffer array
 *
 * @param _arrayOffset Data array offset
 */
void loadMACBuffer(uint16_t _arrayOffset) {
  for ( uint8_t i = 0; i < 6; i++ )
  {
    dataBuffer[_arrayOffset + i] = mac[(5 - i)]; // Reverse byte order to allow sequential retreval on the server
    calcChecksum(mac[(5 - i)]); // Add MAC address to the checksum
  }
}

/**
 * Calculate XOR checksum
 *
 * @param _value Value to add to the running checksum
 */
void calcChecksum(uint8_t _value) {
  checksum = checksum ^ _value;
}

/**
 * Send the data buffer to the postURL set in env.h
 */
// void httpPost() {
//   if ( WiFi.status() == WL_CONNECTED ) // Check WiFi connection status
//   {
//     HTTPClient httpClient;
//
//     httpClient.begin(postURL); // Specify destination for HTTP request
//     httpClient.addHeader(F("Content-Type"), F("application/json")); // Specify content-type
//     httpClient.addHeader(F("Content-Length"), String(BUFFER_SIZE)); // Specify content length
//
//     uint16_t httpResponseCode = httpClient.POST(dataBuffer, BUFFER_SIZE); // Send the POST request
//
//     if( httpResponseCode > 0 )
//     {
//       display.println(httpResponseCode);
//       display.display();
//     } else {
//       display.clearDisplay();
//       display.setCursor(0,0);
//       display.print(F("HTTP Err: "));
//       display.println(dataBuffer[CHECKSUM_OFFSET]);
//       display.display();
//     }
//
//     httpClient.end(); // Free resources
//   } else {
//      display.clearDisplay();
//      display.setCursor(0,0);
//      display.print(F("No WiFI"));
//      display.display();
//      connectToWiFi();
//   }
// }

void wsSendBuffer() {
  if ( (WiFi.status() == WL_CONNECTED) && (isConnected = true) ) // Verify WiFi and WebSocket connection status
  {
    uint8_t * bufferPointer = (uint8_t *) &dataBuffer[0]; // Create pointer to dataBuffer
    webSocket.sendBIN(bufferPointer, BUFFER_SIZE);
    display.println("OK");
    display.display();
  } else {
    // TODO handle non-connected state
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
	switch(type) {
		case WStype_DISCONNECTED:
			DEBUG_MSG("[WS] Disconnected\n");
      isConnected = false;
			break;

		case WStype_CONNECTED: {
			DEBUG_MSG("[WS] Connected\n");
      webSocket.sendTXT("{\"c\":\"sub\",\"chan\":\"hb\"}"); // Subscribe to "hb" HeartBeat channel
      webSocket.sendTXT("{\"c\":\"sub\",\"chan\":\"dormio\"}"); // Subscribe to "dormio" channel
      isConnected = true;
		  } break;

		case WStype_TEXT:
			DEBUG_MSG("[WS] received text: %s\n", payload);

      if ( length == 1 ) {  // Handle single character commands
        uint8_t val = strtol((const char *)payload, NULL, 0); // Convert payload char to it's binary value representation

        switch (val) {
          case 1:
            webSocket.disconnect();
            delay(100);
            WiFi.mode(WIFI_OFF);
            delay(3000);
            ESP.restart();
            break;

          case 2:
            webSocket.sendTXT("2");
            DEBUG_MSG("[WS] Sent heartbeat response\n");
            // TODO Heartbeat: log that the response was received with time and latency(maybe), (handle non responders in the heartbeat script)
            break;

          case 9:
            checkForUpdates();
            break;

          default:
            DEBUG_MSG("[WS] Invalid command\n");
        }

      }
      break;

		case WStype_BIN:
			DEBUG_MSG("[WS] get binary length: %u\n", length);
			hexdump(payload, length);
			break;

    case WStype_FRAGMENT_TEXT_START:
      DEBUG_MSG("[WS] FRAGMENT_TEXT_START %s\n", payload);
      break;

    case WStype_FRAGMENT_BIN_START:
      DEBUG_MSG("[WS] FRAGMENT_BIN_START %s\n", payload);
      break;

    case WStype_FRAGMENT:
      DEBUG_MSG("[WS] FRAGMENT %s\n", payload);
      break;

    case WStype_FRAGMENT_FIN:
      DEBUG_MSG("[WS] FRAGMENT_FIN %s\n", payload);
      break;

    case WStype_ERROR:
      DEBUG_MSG("[WS] Error %s\n", payload);
      break;

    case WStype_PING:
      DEBUG_MSG("[WS] Error %s\n", payload);
      break;

    case WStype_PONG:
      DEBUG_MSG("[WS] Error %s\n", payload);
      break;
    
    default:
      DEBUG_MSG("[WS] Unhandled response type %s\n", payload);
      break;
	} // END Switch WStype
}

/**
 * Encrypt the data buffer using the encryptionKey array set in env.h
 */
void encryptBuffer() {
  for(uint8_t key = 0; key < sizeof(encryptionKey); key++) { // Loop through each encryption key
    for(uint16_t data = 0; data < BUFFER_SIZE; data++) { // Loop through each byte in the data buffer
      dataBuffer[data] = dataBuffer[data] ^ encryptionKey[key]; // XOR the current byte with the encryption key
    }
  }
}

/**
 * Activates one of the eight I2C channels available on the TCA9548A
 *
 * @param channel Select a channel from 0-7
 */
void I2CSelect(uint8_t _channel) {
  if (_channel > 7) return;

  Wire.beginTransmission(0x70);
  Wire.write(1 << _channel);
  Wire.endTransmission();
}

/**
 * Check for a new firmware version and flash OTA if one is available
 */
void checkForUpdates() {
  String fwURL = String( fwUrlBase ); // Get firmware URL base
  String fwVersionURL = fwURL;
  fwVersionURL.concat( F("ver/") ); // Build version request URL
  String encodedMAC = getFormatedMAC();
  fwVersionURL.concat( encodedMAC );

  display.clearDisplay();
  display.setCursor(0,0);
  display.print( F("Build# ") );
  display.println( firmwareVersion );
  display.println( F("Checking For Updates") );
  display.display();
  DEBUG_MSG("Checking for updates\n");

  HTTPClient httpClient;
  httpClient.begin( wifiClient, fwVersionURL ); // Specify destination for HTTP request
  int httpCode = httpClient.GET(); // Send the GET request
  if( httpCode == 200 ) {
    String newFWVersion = httpClient.getString(); // Get current version according to the update server
    uint16_t newVersion = newFWVersion.toInt();

    if( newVersion > firmwareVersion ) { // If there is a newer version then flash OTA
      DEBUG_MSG("Update exists, do update\n");
      display.print( F("Updating From Version") );
      display.print( firmwareVersion );
      display.print( F(" to ") );
      display.println( newFWVersion );
      display.display();

      String fwImageURL = ( F("/bin/") );
      fwImageURL.concat( encodedMAC );
      WiFiClient client;
      t_httpUpdate_return ret = ESPhttpUpdate.update( client, updateHost, updatePort, fwImageURL ); // Atempt to do update

      display.print( F("Update Error: ") );
      display.println( ret );
      display.println( ESPhttpUpdate.getLastError() );
      display.display();
    }
    else {
      DEBUG_MSG("No Updates\n");
      display.println( F("No Updates") );
      display.display();
    }
  }
  else {
    DEBUG_MSG("Update version check error: %i\n", httpCode);
    display.print( F("Version Check Failed Error: ") );
    display.println( httpCode );
    display.display();
  }
  httpClient.end(); // Free resources
}

/**
 * Get zero padded MAC address string
 *
 * @return string Formated MAC
 */
String getFormatedMAC()
{
  char result[14];

  snprintf( result, sizeof( result ), "%02x%02x%02x%02x%02x%02x", mac[ 5 ], mac[ 4 ], mac[ 3 ], mac[ 2 ], mac[ 1 ], mac[ 0 ] );
  return String( result );
}

/**
 * Connect to existing WiFi credentials or start an access point and load a captive portal to configure a connection
 *
 * @param resetSettings If set to true calling this function clears any existing WiFi credentials
 */
void connectToWiFi(boolean resetSettings) {
  DEBUG_MSG("Connectiong to WiFi\n");
  WiFiManager wifiManager; // Local WiFiManager Initialization

  if (resetSettings) wifiManager.resetSettings(); // If flag is set reset wifi credentials

  wifiManager.setAPCallback(WiFiConfigMode); // Set config mode callback
  // wifiManager.setSaveConfigCallback(WiFiSuccess); // Set success callback

  wifiManager.setBreakAfterConfig(true); // Exit after config, even if connection is unsuccessful

  // Attempt to connect to last known WiFi credentials
  // If none exist or the connection fails start an access point name "Dormio Setup"
  // Enter into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("Dormio Setup")) {
    delay(3000);
    ESP.reset();
  }
}

/**
 * Called when "Dormio Setup" access point and captive configuration portal are ready
 * Updates OLED display with instructions
 */
void WiFiConfigMode(WiFiManager *myWiFiManager) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("WiFi Setup Mode"));
  display.println(F("Connect To SSID:"));
  display.println();
  display.println("\"" + myWiFiManager->getConfigPortalSSID() + "\"");
  display.display();
}

/**
 * Called when connection to WiFi was successful and updates OLED display with connection info
 */
void WiFiSuccess() {
  DEBUG_MSG("Wifi Success\n");
  WiFi.macAddress(mac); // Read MAC address into mac array
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Connected To:"));
  display.println(WiFi.SSID());
  display.print(F("IP "));
  display.println(WiFi.localIP());
  display.print(F("MAC "));
  display.print(mac[5],HEX);
  display.print(F(":"));
  display.print(mac[4],HEX);
  display.print(F(":"));
  display.print(mac[3],HEX);
  display.print(F(":"));
  display.print(mac[2],HEX);
  display.print(F(":"));
  display.print(mac[1],HEX);
  display.print(F(":"));
  display.println(mac[0],HEX);
  display.display();
}

/**
 * Establish persistent WebSocket connection to Dormio server
 */
void connectToWebSocket() {
  DEBUG_MSG("Connectiong to Dormio websocket server\n");
  display.clearDisplay();
  display.setCursor(0,9);
  display.println( F("    Connecting to") );
  display.println( F("    Dormio Server") );
  display.display();

  webSocket.begin(wsHost, wsPort, wsAccessToken, ""); // Connect to WebSocket server
  webSocket.onEvent(webSocketEvent); // Set Websocket event handler
  webSocket.setReconnectInterval(10000); // Try to reconnect every 10 seconds if connection has failed
}

/**
 * Handles button press to toggle OLED display and reset WiFi connection
 */
void wifiResetButton() {
  wifiBtn->read();

  if (wifiBtn->wasPressed()) {
    DEBUG_MSG("WiFi button pressed\n");
    pressedAtMillis = millis();
  }

  if (wifiBtn->wasReleased()) {
    DEBUG_MSG("WiFi button released\n");
    if (pressedForMillis > longPressInterval) {
      // Long press, reset WiFi credentials and start config portal AP
      displayOn();
      display.clearDisplay();
      display.setCursor(0,9);
      display.invertDisplay(true);
      display.println("   Resetting WiFi");
      display.display();
      delay(1000);
      display.invertDisplay(false);
      connectToWiFi(true);
    } else {
      // Short press, toggle OLED display on and off
      if (displayStatus) {
        displayOff();
      } else {
        displayOn();
      }
    }
  }
  pressedForMillis = millis() - pressedAtMillis;
}

/**
 * Toggle OLED display off
 */
void displayOff() {
  display.ssd1306_command(0xAE);
  displayStatus = false;
  DEBUG_MSG("Display off\n");
}

/**
 * Toggle OLED display on
 */
void displayOn() {
  display.ssd1306_command(0xAF);
  displayStatus = true;
  DEBUG_MSG("Display on\n");
}
