#include "main.h"

/**
 * Initialization routines
 */
void setup() {
  Serial.begin(115200);                // Initialize serial data transmission and set baud rate

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
  display.println(F(" Dormio Device Duo"));
  display.println(F(" --==Deep Dream==--"));
  display.println(F(" Connecting to WiFi"));
  display.display();
  WiFi.macAddress(mac); // Read MAC address into mac array
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    display.print(F("."));
    display.display();
  }
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Connected OK"));
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
 * Main program loop
 */
void loop() {
  sampleRateFull(); // Full sample rate (50 per second)

  if ( fullSampleCounter % HALF_SF == 0) sampleRateModHalfSF(); // SAMPLE_COUNT/HALF_SF (2 per second)

  if ( fullSampleCounter == SAMPLE_COUNT ) {
    sampleRateSingle(); // Minimum sample rate (1 every 5 seconds)
    encryptBuffer(); // Encrypt the data buffer before sending
    httpPost(); // Sample frame complete, send data
    fullSampleCounter = 0; // Frame complete, reset counters
    modSFSampleCounter = 0;
    checksum = 0; // Reset checksum
  }
}

/**
 * Function holding routines excecuted at the maximum sampling frequency per frame
 */
void sampleRateFull() {
  // Load IMU redings to data buffer
  load32Buffer(ppg.getIR(), fullSampleCounter, IR_OFFSET);
  load32Buffer(ppg.getRed(), fullSampleCounter, RED_OFFSET);
  // Load ECG reading to data buffer
  load16Buffer(ads.readADC_SingleEnded(ECGpin), fullSampleCounter, ECG_OFFSET);

  ppg.nextSample();
  fullSampleCounter++;
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
void httpPost() {
  if( WiFi.status() == WL_CONNECTED ) // Check WiFi connection status
  {
    HTTPClient httpClient;

    httpClient.begin(postURL); // Specify destination for HTTP request
    httpClient.addHeader(F("Content-Type"), F("application/json")); // Specify content-type
    httpClient.addHeader(F("Content-Length"), String(BUFFER_SIZE)); // Specify content length

    uint16_t httpResponseCode = httpClient.POST(dataBuffer, BUFFER_SIZE); // Send the POST request

    if( httpResponseCode > 0 )
    {
      display.println(httpResponseCode);
      display.display();
      I2CSelect(6); // Select I2C bus channel 6
      haptic.go(); // Play the effect
      I2CSelect(7); // Select I2C bus channel 7
    } else {
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(F("HTTP Err: "));
      display.println(dataBuffer[CHECKSUM_OFFSET]);
      display.display();
    }

    httpClient.end(); // Free resources
  } else {
     display.clearDisplay();
     display.setCursor(0,0);
     display.print(F("No WiFI"));
     display.display();
  }
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
