#include "main.h"


void setup() {
  Serial.begin(115200);                // Initialize serial data transmission and set baud rate
  Serial.println("Begin");

  Wire.begin();                        // Initiate the Wire library and join the I2C bus as master

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

  particleSensor.begin(Wire, I2C_SPEED_FAST);
  // Configure MAX3010X sensor with these settings
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  /*END*********MAX30102 Pulse Oximeter and Heart-Rate Sensor******************/

  Serial.println("Setup Complete");
}


void loop() {
    Serial.println("Program Loop");

    getNextHeartRateSample();

    doHttpPost();

    delay(5000);
}


void getNextHeartRateSample(void) {
  redBuffer[hrBufferCounter] = particleSensor.getRed();
  irBuffer[hrBufferCounter] = particleSensor.getIR();
  particleSensor.nextSample();
  hrBufferCounter++;
}


void doHttpPost(void) {
  Serial.println("Starting WiFi");
    WiFi.enableSTA(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) { //Check for the connection
      delay(250);
      Serial.println("Connecting..");
    }

    Serial.print("Connected to the WiFi network with IP: ");
    Serial.println(WiFi.localIP());

    if(WiFi.status()== WL_CONNECTED){   //Check WiFi connection status

      HTTPClient http;

      http.begin(post_url);  //Specify destination for HTTP request
      http.addHeader("Content-Type", "application/json");             //Specify content-type header

      // Make some test data
      String secondsSinceBoot = String(millis()/1000);
      // Send the actual POST request
      int httpResponseCode = http.POST("{ \"seconds-since-boot\": " + String(secondsSinceBoot) + ", \"data\": {\"test\": " + 123 + "} }");

      if(httpResponseCode>0){

        Serial.println(httpResponseCode);   //Print return code

      }else{

       Serial.print("Error on sending request: ");
       Serial.println(httpResponseCode);

      }

      http.end();  //Free resources

    }else{

       Serial.println("WiFi connection error");

    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi disconnected");
}
