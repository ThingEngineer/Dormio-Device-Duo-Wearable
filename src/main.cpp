#include "main.h"


void setup() {
  Serial.begin(115200);
  Serial.println("Begin");

  /***********************************ESP32*************************************/
  WiFi.disconnect(true);               // Initial state - Wifi disabled
  WiFi.mode(WIFI_OFF);
  /*END*******************************ESP32*************************************/

  Serial.println("Setup Complete");
}


void loop() {
    Serial.println("Program Loop");

    doHttpPost();

    delay(5000);
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
