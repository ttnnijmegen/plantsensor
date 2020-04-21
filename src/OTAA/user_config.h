// -- Project -------------------------------------------
#define CLIENT                 "xxx"          // Client ID for the ESP

// -- Wifi ----------------------------------------------
#define WIFI_SSID              "xxx"             // [Ssid1] Wifi SSID
#define WIFI_PASS              "xxx"         // [Password1] Wifi password


// -- DHT - Sensor --------------------------------------
#define DHTTYPE                 DHT11                 // [Sensor] Change to DHT11, DHT21, DHT22
#define PIN_DHT                 22                    // [Sensor] Pin for DHT sensor

// -- Other - Sensor ------------------------------------
#define PIN_SOIL                32                    // [Sensor] Soil Sensor pin
#define PIN_LIGHT               33                    // [Sensor] Light Sensor pin
#define PIN_POWER               34                    // [Sensor] Power Sensor pin


// -- MQTT ----------------------------------------------
#define MQTT                    0                     // [MQTT] Mosquitto (0 = Off, 1 = Start)
#define MQTT_BROKER             "192.168.178.20"             // [MQTT] Set the IP adress from your MQ-Broker
#define MQTT_PORT               1883                  // [MQTT] MQTT Port on the Server
#define MQTT_TOPIC              "xxx"   // [MQTT] Set the topic for your queue
#define MQTT_INTERVAL           3600                  // [MQTT] Set the deepsleep time in seconds


// -- HTTP ---------------------------------------------- When webser is activ deepsleep will be disabled
#define WEB_SERVER              0                     // [WebServer] Web server (0 = Off, 1 = Start)
#define WEB_PORT                80                    // [WebServer] Port for web access


// not yet implemented. 
// -- Cayenne -------------------------------------------
//#define CAYENNE_SUPPORT         1                      // Cayenne support to 
//#define CAYENNE_PRINT Serial 
//#define SENSOR_PIN 5 // Do not use digital pins 0 or 1 since those conflict with the use of Serial.
//#define VIRTUAL_CHANNEL 1
//char username[] = "MQTT_USERNAME";
//char password[] = "MQTT_PASSWORD";
//char clientID[] = "CLIENT_ID";
