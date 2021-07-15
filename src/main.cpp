#include <Arduino.h>
#include <HardwareSerial.h>
#include <LITTLEFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
// #include <ESPAsync_WiFiManager.h>
#include <ESPDash.h>




HardwareSerial mySerial2(2);

unsigned short i;
char temp;


/****************************************************************************************************************************
  Async_ConfigOnDoubleReset_minimal.ino
  For ESP8266 / ESP32 boards
  Built by Khoi Hoang https://github.com/khoih-prog/ESPAsync_WiFiManager
  Licensed under MIT license
 *****************************************************************************************************************************/
#if !(defined(ESP32) )
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

#if defined(ESP32)
  #define USE_SPIFFS            true
  #define ESP_DRD_USE_EEPROM    true
#else  
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.  
#endif
#include <ESPAsync_WiFiManager.h>               //https://github.com/khoih-prog/ESPAsync_WiFiManager
#define DRD_TIMEOUT             10
#define DRD_ADDRESS             0
#include <ESP_DoubleResetDetector.h>            //https://github.com/khoih-prog/ESP_DoubleResetDetector
DoubleResetDetector* drd;
const int PIN_LED       = 2;
bool      initialConfig = false;
AsyncWebServer webServer(80);
#if !( USING_ESP32_S2 || USING_ESP32_C3 )
DNSServer dnsServer;
#endif

/* Attach ESP-DASH to AsyncWebServer */
ESPDash dashboard(&webServer);

/* 
  Dashboard Cards 
  Format - (Dashboard Instance, Card Type, Card Name, Card Symbol(optional) )
*/
Card* temperature = new Card(&dashboard, TEMPERATURE_CARD, "Temperature", "°C");
Card* humidity = new Card(&dashboard, HUMIDITY_CARD, "Humidity", "%");

/* 
  Removal Time for demonstration purposes only
  In your final project,
  You will decide when to add or remove cards, these variables are not requried.
*/
unsigned long removalTime = 30000; // Remove after 30s ( 30000 Millis )
bool cardRemoved = false;


void setup() {
  webServer.on("/up", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! I am ESP32.");
    });

  // server.begin();
  // Serial.println("HTTP server started");
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); while (!Serial); delay(200);
  mySerial2.begin(9600, SERIAL_8N1, 26, 27);
  Serial.print(F("\nStarting Async_ConfigOnDoubleReset_minimal on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION); 
  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  if (drd->detectDoubleReset()) { Serial.println(F("DRD")); initialConfig = true; }
#if ( USING_ESP32_S2 || USING_ESP32_C3 )
  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, NULL, "Async_ConfigOnDRD");
#else
  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "Async_ConfigOnDRD");
#endif
  if (ESPAsync_wifiManager.WiFi_SSID() == "") { Serial.println(F("No AP credentials")); initialConfig = true; }
  if (initialConfig) {
    Serial.println(F("Starting Config Portal")); digitalWrite(PIN_LED, HIGH);
    if (!ESPAsync_wifiManager.startConfigPortal()) { Serial.println(F("Not connected to WiFi")); }
    else { Serial.println(F("connected")); }
  }
  else { WiFi.mode(WIFI_STA); WiFi.begin(); }
  digitalWrite(PIN_LED, LOW); 
  unsigned long startedAt = millis();
  Serial.print(F("After waiting "));
  int connRes = WiFi.waitForConnectResult();
  float waited = (millis() - startedAt);
  Serial.print(waited / 1000); Serial.print(F(" secs , Connection result is ")); Serial.println(connRes);
  if (WiFi.status() != WL_CONNECTED) { Serial.println(F("Failed to connect")); }
  else { Serial.print(F("Local IP: ")); Serial.println(WiFi.localIP()); }
  
  AsyncElegantOTA.begin(&webServer);    // Start ElegantOTA

}
void loop() { 
  drd->loop(); 
  /* Update Card Values */
  if(temperature != nullptr) // Check if our pointer has not been deleted & then only access 'update' method
    temperature->update((int)random(0, 50));

  if(humidity != nullptr) // Check if our pointer has not been deleted & then only access 'update' method
    humidity->update((int)random(0, 100));

  
  /* Remove our card when removal time is elapsed */
  if(millis() > removalTime && !cardRemoved){
    if(humidity != nullptr){
      /* Calling 'delete' will remove this card from our dashboard instance and will free-up consumed heap */
      delete humidity; 
      humidity = nullptr; // Make sure to set this pointer to 'nullptr' when deleted 
    }
    // Set our removal flag
    cardRemoved = true;
  }

  /* Send Updates to our Dashboard (realtime) */
  dashboard.sendUpdates();

  /* 
    Delay is just for demonstration purposes in this example,
    Replace this code with 'millis interval' in your final project.
  */
  delay(100);


}


void clear_uart_buffer() {
  i = mySerial2.available();
  if(i != 0) {
    while(i--) {
      mySerial2.read();
    }
  }
}

void read_uart(){
  i = mySerial2.available();
  if(i != 0) {
    while(i--) {
      temp = mySerial2.read();
    }
  }
  Serial.print(temp);
  // temp = 
}


// void loop() {
//   // put your main code here, to run repeatedly:
//   //  myserial2.read();
//   // read_uart();
//   while(mySerial2.available()) {
//     Serial.print(char(mySerial2.read()));
//   }
//   delay(100);
// }