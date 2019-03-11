// -- Project -------------------------------------------
#define CLIENT                 "plantsensor01"          // Client ID for the ESP

// -- Wifi ----------------------------------------------
#define WIFI_SSID              "<your ssid>"             // [Ssid1] Wifi SSID
#define WIFI_PASS              "<your password>"         // [Password1] Wifi password


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
#define MQTT_TOPIC              "soilmoisture"   // [MQTT] Set the topic for your queue
#define MQTT_INTERVAL           3600                  // [MQTT] Set the deepsleep time in seconds


// -- HTTP ---------------------------------------------- When webser is activ deepsleep will be disabled
#define WEB_SERVER              0                     // [WebServer] Web server (0 = Off, 1 = Start)
#define WEB_PORT                80                    // [WebServer] Port for web access








