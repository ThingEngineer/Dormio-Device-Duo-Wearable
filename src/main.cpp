#include "main.h"


void setup() {
  Serial.begin(115200);                // Initialize serial data transmission and set baud rate

  Wire.begin(4,5);
  tempSensor.begin();                  // Initilize MLX90614 temperature sensor
  IMU.begin();                         // Initialize LSM6DS3 6DOF IMU
  // hapticFeedback.begin();              // Initialize DRV2605 haptic feedback driver
  // hapticFeedback.setMode(DRV2605_MODE_INTTRIG);
  // hapticFeedback.selectLibrary(1);     // Set haptic feedback library
  // hapticFeedback.setWaveform(1, 1);    // Strong click 100%, see datasheet part 11.2

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
  display.setCursor(0,9);
  display.println(F("  Dormio Device Duo"));
  display.println(F("  --==Deep Dream==--"));
  display.display();
  delay(1000);
  /*END********************************OLED***********************************/

  /***********************************ESP8266*********************************/
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Connecting to WiFi"));
  display.display();
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
  display.println(F("IP Address:"));
  display.println(WiFi.localIP());
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
  dataBuffer[CHECKSUM_OFFSET] = checksum; // Load checksum to data buffer

  display.clearDisplay();
  display.setCursor(0,0);
  // display.println((millis() / 1000));
  display.println(frameCounter);
  // display.println(dataBuffer[CHECKSUM_OFFSET]);
  display.display();

  frameCounter++;
}


  uint16_t _byteOffset;
  uint8_t _shiftOffset;
  for ( int i = 0; i < 4; i++ ) // Load sensor data into data buffer array (Little endian unsigned long to unsigned char[4])
  {
    _byteOffset = ((hrSampleCounter * 4) + i);
    _shiftOffset = (8 * i);

    dataBuffer[IR_OFFSET + _byteOffset] = ( irbufferTemp >> _shiftOffset );
    checksum = checksum ^ dataBuffer[IR_OFFSET + _byteOffset];

    dataBuffer[RED_OFFSET + _byteOffset] = ( redbufferTemp >> _shiftOffset );
    checksum = checksum ^ dataBuffer[RED_OFFSET + _byteOffset];
  }

  hrSensor.nextSample();
  hrSampleCounter++;
}


void sampleRateModSF() {

}


void sampleRateSingle() {

}


void httpPost() {
  if( WiFi.status() == WL_CONNECTED ) // Check WiFi connection status
  {
    HTTPClient http;

    http.begin(post_url); // Specify destination for HTTP request
    http.addHeader(F("Content-Type"), F("application/json")); // Specify content-type
    http.addHeader(F("Content-Length"), String(BUFFER_SIZE)); // Specify content length

    uint16_t httpResponseCode = http.POST(dataBuffer, BUFFER_SIZE); // Send the POST request

    if( httpResponseCode > 0 )
    {
      display.println(httpResponseCode);
      display.display();
      // hapticFeedback.go();                 // Play the effect
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
