#include "main.h"


void setup() {
  Serial.begin(115200);                // Initialize serial data transmission and set baud rate
  Serial.println(F("Initializing"));

  Wire.begin(21,22);                   // Initiate the Wire library and join the I2C bus as master

  /***********************************ESP32*************************************/
  WiFi.disconnect(true);               // Initial state - Wifi disabled
  WiFi.mode(WIFI_OFF);

  /*END*******************************ESP32*************************************/

  /*************MAX30102 Pulse Oximeter and Heart-Rate Sensor******************/
  byte ledBrightness = 32; // Options: 0=Off to 255=50mA
  byte sampleAverage = 8; // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 800; // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 215; // Options: 69, 118, 215, 411
  int adcRange = 2048; // Options: 2048, 4096, 8192, 16384

  if ( hrSensor.begin(Wire, I2C_SPEED_FAST) == false ) Serial.println(F("MAX30102 Error"));
  // Configure MAX3010X sensor with these settings
  hrSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  redPulseAmplitude = 32;   // Initial red LED pulse amplitude
  /*END*********MAX30102 Pulse Oximeter and Heart-Rate Sensor******************/

  tempSensor.begin();                  // Initilize MLX90614 temperature sensor
  IMU.begin();                         // Initialize LSM6DS3 6DOF IMU

  pinMode(25, OUTPUT);                 // ocilliscope loop speed test ping

  delay(1000);
  Serial.println(F("Setup Complete"));
}


void loop() {

  Serial.print(tempSensor.readAmbientTempF(), DEC);
  Serial.print(F(","));
  Serial.print(tempSensor.readObjectTempF(), DEC);

  Serial.print(F(","));
  Serial.print(IMU.readFloatAccelX(), 4);
  Serial.print(F(","));
  Serial.print(IMU.readFloatAccelY(), 4);
  Serial.print(F(","));
  Serial.print(IMU.readFloatAccelZ(), 4);
  Serial.print(F(","));
  Serial.print(IMU.readFloatGyroX(), 4);
  Serial.print(F(","));
  Serial.print(IMU.readFloatGyroY(), 4);
  Serial.print(F(","));
  Serial.print(IMU.readFloatGyroZ(), 4);
  Serial.print(F(","));
  Serial.print(IMU.readTempC(), 4);
  Serial.print(F(","));
  Serial.println(IMU.readTempF(), 4);

  getNextHeartRateSample();
  if ( hrBufferCounter == SF ) normalizeRedLED();
  if ( hrBufferCounter == BUFFER_SIZE ) doHttpPost();
  digitalWrite( 25, !digitalRead(25) );

}


void getNextHeartRateSample() {
  redBuffer[hrBufferCounter] = hrSensor.getRed();
  irBuffer[hrBufferCounter] = hrSensor.getIR();
  hrSensor.nextSample();
  hrBufferCounter++;
}


void normalizeRedLED() {
  // Calculate Red & IR DC mean
  redMean = 0;
  irMean = 0;
  int _hrBufferCounter = 0;
  for (_hrBufferCounter = 0 ; _hrBufferCounter < SF ; _hrBufferCounter++) {
    redMean += redBuffer[_hrBufferCounter];
    irMean += irBuffer[_hrBufferCounter];
  }
  redMean = redMean/SF;
  irMean = irMean/SF;

  // Adjust red LED current if it is to far above or below IR LED reading
  if ( irMean > (redMean + 8000) ) redPulseAmplitude++;
  if ( irMean < (redMean - 8000) ) redPulseAmplitude--;

  if ( redPulseAmplitudePrevious != redPulseAmplitude ) {

    hrSensor.setPulseAmplitudeRed(redPulseAmplitude);   // Set new red LED current
    hrBufferCounter = 0;    // If we had to make a change, red current is not stable this round, reset buffer

    Serial.println(F("Normalizing Red LED"));

  }
  redPulseAmplitudePrevious = redPulseAmplitude;
}


void doHttpPost() {
  Serial.println(F("Starting WiFi"));
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) { //Check for the connection

    delay(250);
    Serial.println(F("Connecting.."));

  }

  Serial.print(F("Connected to the WiFi network with IP: "));
  Serial.println(WiFi.localIP());

  if( WiFi.status() == WL_CONNECTED ) { //Check WiFi connection status

    HTTPClient http;

    http.begin(post_url);  //Specify destination for HTTP request
    http.addHeader("Content-Type", "application/json");             //Specify content-type header

    // Make some test data
    String secondsSinceBoot = String(millis()/1000);
    // Send the actual POST request
    int httpResponseCode = http.POST("{ \"seconds-since-boot\": " + String(secondsSinceBoot) + ", \"data\": {\"test\": " + 123 + "} }");

    if( httpResponseCode > 0 ) {

      Serial.println(httpResponseCode);   //Print return code

    } else {

     Serial.print(F("Error on sending request: "));
     Serial.println(httpResponseCode);

    }

    http.end();  //Free resources

  } else {

     Serial.println(F("WiFi connection error"));

  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println(F("WiFi disconnected"));
}
