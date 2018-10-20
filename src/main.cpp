#include "main.h"


void setup() {
  Serial.begin(115200);                // Initialize serial data transmission and set baud rate
  Serial.println(F("Initializing"));

  Wire.begin(21,22);                   // Initiate the Wire library and join the I2C bus as master

  tempSensor.begin();                  // Initilize MLX90614 temperature sensor
  IMU.begin();                         // Initialize LSM6DS3 6DOF IMU
  hapticFeedback.begin();              // Initialize DRV2605 haptic feedback driver
  hapticFeedback.selectLibrary(1);     // Set haptic feedback library

  /**********************************MAX30102*********************************/
  byte ledBrightness = 32; // Options: 0=Off to 255=50mA
  byte sampleAverage = 8; // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 800; // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 215; // Options: 69, 118, 215, 411
  int adcRange = 2048; // Options: 2048, 4096, 8192, 16384

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

  pinMode(25, OUTPUT);                 // ocilliscope loop speed test ping

  delay(500);
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

  // Serial.print(F(","));
  // Serial.print(IMU.readFloatAccelX(), 4);
  // Serial.print(F(","));
  // Serial.print(IMU.readFloatAccelY(), 4);
  // Serial.print(F(","));
  // Serial.print(IMU.readFloatAccelZ(), 4);
  // Serial.print(F(","));
  // Serial.print(IMU.readFloatGyroX(), 4);
  // Serial.print(F(","));
  // Serial.print(IMU.readFloatGyroY(), 4);
  // Serial.print(F(","));
  // Serial.print(IMU.readFloatGyroZ(), 4);
  // Serial.print(F(","));
  // Serial.print(IMU.readTempC(), 4);
  // Serial.print(F(","));
  // Serial.println(IMU.readTempF(), 4);

  getNextHeartRateSample();
  if ( hrBufferCounter == SF ) normalizeRedLED();
  if ( hrBufferCounter == BUFFER_SIZE ) httpPost();

  // hapticFeedback.setWaveform(1, 1);    // Strong click 100%, see datasheet part 11.2
  // hapticFeedback.go();                 // Play the effect

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

    Serial.println("Normalizing Red LED " + String(redPulseAmplitude));

  }
  redPulseAmplitudePrevious = redPulseAmplitude;
}


void httpPost() {
  // Create some test data
  String secondsSinceBoot = String(millis()/1000);
  String PostData = "{ \"seconds-since-boot\": " + String(secondsSinceBoot) + ", \"data\": {\"test\": " + 123 + "} }";

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
    http.addHeader("Content-Type", "application/json"); // Specify content-type
    http.addHeader("Content-Length", String(PostData.length())); // Specify content length

    int httpResponseCode = http.POST(PostData); // Send the POST request
    http.writeToStream(&Serial);

    if( httpResponseCode > 0 )
    {
      Serial.println(httpResponseCode); // Print return code
    } else {
     Serial.print(F("Error: "));
     Serial.println(httpResponseCode);
    }

    http.end(); // Free resources
  } else {
     Serial.println(F("No WiFi"));
  }

  // WiFi.disconnect(true); // Disable WiFi
  // WiFi.mode(WIFI_OFF);
  // Serial.println(F("WiFi Off"));
}
