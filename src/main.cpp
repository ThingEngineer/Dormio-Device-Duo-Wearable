#include "main.h"


void setup() {
  Serial.begin(115200);                // Initialize serial data transmission and set baud rate
  Serial.println(F("Initializing"));

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
  /*END******************************MAX30102*********************************/

  /***********************************ESP8266*********************************/
  Serial.print("WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print(F("OK - IP address: "));
  Serial.println(WiFi.localIP());
  /*END*******************************ESP8266*********************************/

  /************************************OLED***********************************/
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,9);
  display.println(F("Dormio Device Duo"));
  display.println(F("--==Deep Dream==--"));
  display.display();
  /*END********************************OLED***********************************/


  pinMode(GSRpin, INPUT);

  pinMode(25, OUTPUT); // Ocilliscope loop speed test ping

  Serial.println(F("Setup Complete"));
}


void loop() {
  // Serial.print(tempSensor.readAmbientTempF());
  // Serial.print(F(","));
  // Serial.println(tempSensor.readObjectTempF());

  // Serial.print(F(","));
  // Serial.print(hrSensor.getIR());
  // Serial.print(F(","));
  // Serial.print(hrSensor.getRed());
  if ( fullSampleCounter % HALF_SF == 0) sampleRateModHalfSF(); // SAMPLE_COUNT / HALF_SF (2 per second)

  if ( fullSampleCounter % HALF_SF == 0) sampleRateModHalfSF(); // SAMPLE_COUNT / HALF_SF (2 per second)

  sampleRateFull(); // Full sample rate

  if ( hrSampleCounter % SF == 0) sampleRateModSF(); // Sample rate / SF

  if ( hrSampleCounter == SAMPLE_COUNT ) {
    sampleRateSingle(); // Minimum sample rate
    httpPost(); // Sample cycle complete, send data
  }


  digitalWrite( 25, !digitalRead(25) );
}


void sampleRateFull() {
  uint32_t irbufferTemp, redbufferTemp; // Get new ir and red sensor readings
  irbufferTemp = hrSensor.getIR();
  redbufferTemp = hrSensor.getRed();

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
  Serial.println(F("Post"));
  hrSampleCounter = 0;

  // Create some test data
  // String secondsSinceBoot = String(millis()/1000);
  // char* jsonData = "{ \"seconds-since-boot\": " + String(secondsSinceBoot) + ", \"data\": {\"test\": " + 123 + "} }";

  // Serial.println(F("WiFi"));
  // WiFi.enableSTA(true); // Enable WiFi and connect
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // while ( WiFi.status() != WL_CONNECTED ) // Check for the connection
  // {
  //   delay(500);
  //   Serial.print(F("."));
  // }

  if( WiFi.status() == WL_CONNECTED ) // Check WiFi connection status
  {
    HTTPClient http;

    http.begin(post_url); // Specify destination for HTTP request
    http.addHeader(F("Content-Type"), F("application/json")); // Specify content-type
    http.addHeader(F("Content-Length"), String(BUFFER_SIZE)); // Specify content length

    uint16_t httpResponseCode = http.POST(dataBuffer, BUFFER_SIZE); // Send the POST request

    if( httpResponseCode > 0 )
    {
      Serial.println(httpResponseCode); // Print return code
    } else {
     Serial.print(F("Error: "));
     Serial.println(httpResponseCode);
      // hapticFeedback.go();                 // Play the effect
    }

    http.end(); // Free resources
  } else {
     Serial.println(F("No WiFi"));
  }

  // WiFi.disconnect(true); // Disable WiFi
  // WiFi.mode(WIFI_OFF);
  // Serial.println(F("WiFi Off"));
}
