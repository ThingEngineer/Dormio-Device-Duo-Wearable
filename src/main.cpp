#include "main.h"


void setup() {
  Serial.begin(115200);                // Initialize serial data transmission and set baud rate

  Wire.begin(4,5);                     // Initial I2C and join bus as master

  I2CSelect(6);                        // Select I2C bus channel 6
  hapticFeedback.begin();              // Initialize DRV2605 haptic feedback driver
  hapticFeedback.setMode(DRV2605_MODE_INTTRIG);
  hapticFeedback.selectLibrary(1);     // Set haptic feedback library
  hapticFeedback.setWaveform(1, 1);    // Strong click 100%, see datasheet part 11.2

  I2CSelect(7);                        // Select I2C bus channel 7
  tempSensor.begin();                  // Initilize MLX90614 temperature sensor
  IMU.begin();                         // Initialize LSM6DS3 6DOF IMU

  /**********************************MAX30102*********************************/
  uint8_t ledBrightness = 32; // Options: 0=Off to 255=50mA
  uint8_t sampleAverage = 8; // Options: 1, 2, 4, 8, 16, 32
  uint8_t ledMode = 2; // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  uint16_t sampleRate = 800; // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  uint16_t pulseWidth = 215; // Options: 69, 118, 215, 411
  uint16_t adcRange = 2048; // Options: 2048, 4096, 8192, 16384

  if ( hrSensor.begin() == false ) Serial.println(F("MAX30102 Error"));
  hrSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  hrSensor.readTemperature(); // Initial temperture reading
  /*END******************************MAX30102*********************************/

  /************************************OLED***********************************/
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  /*END********************************OLED***********************************/

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
  display.print("MAC ");
  display.print(mac[5],HEX);
  display.print(":");
  display.print(mac[4],HEX);
  display.print(":");
  display.print(mac[3],HEX);
  display.print(":");
  display.print(mac[2],HEX);
  display.print(":");
  display.print(mac[1],HEX);
  display.print(":");
  display.println(mac[0],HEX);
  display.display();
  /*END*******************************ESP8266*********************************/

  pinMode(GSRpin, INPUT);
}


void loop() {
  sampleRateFull(); // Full sample rate (50 per second)

  if ( fullSampleCounter % HALF_SF == 0) sampleRateModHalfSF(); // SAMPLE_COUNT / HALF_SF (2 per second)

  if ( fullSampleCounter == SAMPLE_COUNT ) {
    sampleRateSingle(); // Minimum sample rate
    httpPost(); // Sample cycle complete, send data
    fullSampleCounter = 0; // Cycle complete, reset counters
    modSFSampleCounter = 0;
  }
}


void sampleRateFull() {
  load32Buffer(hrSensor.getIR(), fullSampleCounter, IR_OFFSET);
  load32Buffer(hrSensor.getRed(), fullSampleCounter, RED_OFFSET);

  hrSensor.nextSample();
  fullSampleCounter++;
}


void sampleRateModHalfSF() {
  // Get and load IMU redings to data buffer
  loadFloatBuffer(IMU.readFloatAccelX(), modSFSampleCounter, ACCEL_X_OFFSET);
  loadFloatBuffer(IMU.readFloatAccelY(), modSFSampleCounter, ACCEL_Y_OFFSET);
  loadFloatBuffer(IMU.readFloatAccelZ(), modSFSampleCounter, ACCEL_Z_OFFSET);
  loadFloatBuffer(IMU.readFloatGyroX(), modSFSampleCounter, GYRO_X_OFFSET);
  loadFloatBuffer(IMU.readFloatGyroY(), modSFSampleCounter, GYRO_Y_OFFSET);
  loadFloatBuffer(IMU.readFloatGyroZ(), modSFSampleCounter, GYRO_Z_OFFSET);

  load16Buffer(analogRead(GSRpin), modSFSampleCounter, GSR_OFFSET); // Get and load GSR reding to data buffer

  modSFSampleCounter++;
}


void sampleRateSingle() {
  loadFloatBuffer(hrSensor.readTemperature(), 0, DIE_TEMP_OFFSET); // Load current MAX30102 die temperture to data buffer
  loadFloatBuffer(tempSensor.readObjectTempC(), 0, SKIN_TEMP_OFFSET); // Load current skin temperture to data buffer
  loadFloatBuffer(tempSensor.readAmbientTempC(), 0, AMBIENT_TEMP_OFFSET); // Load current ambient temperture to data buffer
  load32Buffer((millis() / 1000), 0, EPOCH_OFFSET); // Load current epoch to data buffer
  load32Buffer(frameCounter, 0, FRAME_COUNT_OFFSET); // Load current frame count to data buffer
  loadMACBuffer(MAC_OFFSET); // Load MAC address into data buffer
  dataBuffer[CHECKSUM_OFFSET] = checksum; // Load checksum to data buffer

  display.clearDisplay();
  display.setCursor(0,0);
  // display.println((millis() / 1000));
  display.println(frameCounter);
  // display.println(dataBuffer[CHECKSUM_OFFSET]);
  display.display();

  frameCounter++;
}


void loadFloatBuffer(float _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset) {
  int16_t characteristic = (int16_t)(_bufferTemp); // Remove the mantissa retaining just a signed integer portion of the value
  if (characteristic < 0.0) characteristic = (characteristic ^ 0x7FFF) + 1; // If negative remove two's complement signing, leave only the significant bit
  // Load the characteristic of the floating point to the data buffer
  uint16_t _byteOffset;
  uint8_t _shiftOffset;
  for ( int i = 0; i < 2; i++ ) // Load sensor data into data buffer array (16 bit Little endian to 8 bit array[2])
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
  calcChecksum(dataBuffer[_arrayOffset + 2]); // Add new buffer value to checksum
}


void load32Buffer(uint32_t _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset) {
  uint16_t _byteOffset;
  uint8_t _shiftOffset;
  for ( int i = 0; i < 4; i++ ) // Load sensor data into data buffer array (32 bit Little endian to 8 bit array[4])
  {
    _byteOffset = ((_sampleCounter * 4) + i);
    _shiftOffset = (8 * i);

    dataBuffer[_arrayOffset + _byteOffset] = ( _bufferTemp >> _shiftOffset );
    calcChecksum(dataBuffer[_arrayOffset + _byteOffset]); // Add new buffer value to checksum
  }
}


void load16Buffer(uint16_t _bufferTemp, uint8_t _sampleCounter, uint16_t _arrayOffset) {
  uint16_t _byteOffset;
  uint8_t _shiftOffset;
  for ( int i = 0; i < 2; i++ ) // Load sensor data into data buffer array (16 bit Little endian to 8 bit array[2])
  {
    _byteOffset = ((_sampleCounter * 2) + i);
    _shiftOffset = (8 * i);

    dataBuffer[_arrayOffset + _byteOffset] = ( _bufferTemp >> _shiftOffset );
    calcChecksum(dataBuffer[_arrayOffset + _byteOffset]); // Add new buffer value to checksum
  }
}


void loadMACBuffer(uint16_t _arrayOffset) {
  for ( uint8_t i = 0; i < 6; i++ ) // Load mac address array into data buffer array
  {
    dataBuffer[_arrayOffset + i] = mac[(5 - i)]; // Reverse byte order to allow sequential retreval on the server
    calcChecksum(mac[i]); // Add MAC address to the checksum
  }
}


void calcChecksum(uint8_t newValue) { // Calculate simple XOR checksum
  checksum = checksum ^ newValue;
}


void httpPost() {
  if( WiFi.status() == WL_CONNECTED ) // Check WiFi connection status
  {
    HTTPClient http;

    encryptBuffer(); // Encrypt the data buffer before sending

    http.begin(post_url); // Specify destination for HTTP request
    http.addHeader(F("Content-Type"), F("application/json")); // Specify content-type
    http.addHeader(F("Content-Length"), String(BUFFER_SIZE)); // Specify content length

    uint16_t httpResponseCode = http.POST(dataBuffer, BUFFER_SIZE); // Send the POST request

    if( httpResponseCode > 0 )
    {
      display.println(httpResponseCode);
      display.display();
      I2CSelect(6);                        // Select I2C bus channel 6
      hapticFeedback.go();                 // Play the effect
      I2CSelect(7);                        // Select I2C bus channel 7
    } else {
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(F("HTTP Err: "));
      display.println(dataBuffer[CHECKSUM_OFFSET]);
      display.display();
    }

    http.end(); // Free resources
  } else {
     display.clearDisplay();
     display.setCursor(0,0);
     display.print(F("No WiFI"));
     display.display();
  }
}


void encryptBuffer() {
  for(uint8_t key = 0; key < sizeof(encryptionKey); key++) { // Loop through each encryption key
    for(uint16_t data = 0; data < BUFFER_SIZE; data++) { // Loop through each byte in the data buffer
      dataBuffer[data] = dataBuffer[data] ^ encryptionKey[key]; // XOR the current byte with the encryption key
    }
  }
}


void I2CSelect(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}
