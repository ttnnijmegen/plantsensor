/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * 
 * Copyright (c) 2018 Roger Meijs LoRaWAN implementation of the ESP32 "HiGrow" soil moisture, temperature and humidity sensor. 
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 * 
 *
 *******************************************************************************/

// to do 
//bluetooth en wifi off with lora ok
//WiFi.mode(WIFI_OFF); ok
//btStop(); ok
//turn leds off
//  Blue led does turn off (not blink) in the ESP32test


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_deep_sleep.h"
#include <DHT.h>                                  // DHT Sensor 
#include "user_config.h"                          // Fixed user configurable options
//#include <CayenneMQTTEthernet.h>                // Cayenne loopt vast ivm ethernet library?
#include <CayenneLPP.h>


// webserver
WiFiServer server(WEB_PORT);

/* create an instance of PubSubClient client */
WiFiClient espClient;
PubSubClient client(espClient);

// Initialize DHT sensor.
DHT dht(PIN_DHT, DHTTYPE);

#define LED_PIN 2

// on off switch of the buildin led
//int LED_BUILTIN = 2;



// ESP32 onboard temp board readout
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();



// Client variables 
long lastMsg = 0;


/////////////////////////////////////////////////////////////////////////
///////////////////////////////// lora keys/////////////////////////////

// This EUI must be in little-endian format (lsb), so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={  };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format (lsb), see above.
static const u1_t PROGMEM DEVEUI[8]={ };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format  (msb) (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//////////////////////////////////////////////////////////////////////////

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping for HiGrow
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {2, 13, 15},
};

#define MAX_SIZE 200
CayenneLPP lpp(MAX_SIZE);

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
 //       byte payload[8];
        // lowpower protocol reset
        lpp.reset();

        // get values ready
        uint8_t water = getWater();
        uint8_t temp = getTemperature();
        uint8_t hum = getHumidity();
        uint8_t HallRead = hallRead();
        uint8_t BoardTemperature = (temprature_sens_read()- 32) / 1.8;

        // place them in LPP context
        lpp.addTemperature(0, temp);
        lpp.addRelativeHumidity(1, hum);
        lpp.addBarometricPressure(2, water);
        lpp.addAnalogOutput(3, HallRead);
        lpp.addTemperature(4, BoardTemperature);

        // send LPP data
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Packet queued"));   

// sending without LPP
//        uint16_t water = getWater();
//        uint16_t temperature = getTemperature()*100;
//        uint16_t humidity = getHumidity()*10;
//        uint16_t light = getLight();
//
//        payload[0] = highByte(water);
//        payload[1] = lowByte(water);
//        payload[2] = highByte(temperature);
//        payload[3] = lowByte(temperature);
//        payload[4] = highByte(humidity);
//        payload[5] = lowByte(humidity);
//        payload[6] = highByte(light);
//        payload[7] = lowByte(light);
//        
//        LMIC_setTxData2(1, payload, 8, 0);
//        Serial.println(F("Packet queued"));

    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// Build JSON - not for sensding with LoRaWAN
String toJson(){
        char body[60];
        sprintf(body, "{\"water\":%d,\"hum\":%d,\"temp\":%d}",
                getWater(), getHumidity(), getTemperature());
 
  return body;
}

// Read the water level and return a readable value
int getWater() {
  int water = analogRead(PIN_SOIL);
  water = map(water, 0, 4095, 0, 1023);
  water = constrain(water, 0, 1023);
  return water;
}

// Read the light level and return a readable value
int getLight() {
  int light = analogRead(PIN_LIGHT);
  light = map(light, 0, 4095, 0, 1023);
  light = constrain(light, 0, 1023);
  return light;
}

// Read the temperature and return a readable value
int getTemperature() {
  return dht.readTemperature();
}
// Read ESP32 internal hall sensor
int HallSensor(){
Serial.println("hall sensor:");
Serial.println(hallRead());
return hallRead();
}
// Read the humidity level and return a readable value
int getHumidity() {
  return dht.readHumidity();
}

void setup() {
    Serial.begin(115200);
      dht.begin();                                    // initialize the DHT sensor
  // define blue light
    pinMode(LED_PIN, OUTPUT);
    digitalWrite (LED_PIN, LOW);
  
    Serial.println(F("Starting"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    #if (WEB_SERVER == 1)  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);               // Connect with Wifi
  
  while(WiFi.status() != WL_CONNECTED) {          // Try to create connection to WiFi
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#elif (WEB_SERVER == 0)  
  WiFi.mode(WIFI_OFF);
  btStop();
  
#endif
  // Config pins
  pinMode(16, OUTPUT); 
  pinMode(PIN_POWER, INPUT);
  digitalWrite(16, LOW);  

  


  if (MQTT == 1) {                                // MQ Start
    client.setServer(MQTT_BROKER, MQTT_PORT);     // Configure the MQTT server
    Serial.println("");
    Serial.print("Set MQTT Broker: ");
    Serial.println(MQTT_BROKER);
    Serial.print("Set MQTT Port: ");
    Serial.println(MQTT_PORT);
  } else {                                        // MQ Stop
    Serial.println("");
    Serial.println("MQTT off");
  }

  if (WEB_SERVER == 1) {                          // Webserver Start
    server.begin();
    Serial.println("Webserver is online");
    Serial.println("Set Webserver Port to: ");
    Serial.println(WEB_PORT);
  } else {                                        // Webserver Stop
    Serial.println("");
    Serial.println("Webserver is offline");
    WiFi.mode(WIFI_OFF);
    Serial.println("WIFI is off (blue light should be out)");
    btStop();
    Serial.println("Bluetooth is off (blue light should be out)");
 }
 // if (CAYENNE_SUPPORT ==1) {                      // Cayenne start
 // Cayenne.begin(username, password, clientID);
}

void loop() {
  digitalWrite (LED_PIN, LOW);
    os_runloop_once();
    // -- MQTT --------------------------------------------
  if (MQTT == 1) {
    if (!client.connected()) {                    // Check the connection, if disconnected try to reconnect
      mqttconnect();
    }

    if (WEB_SERVER == 1) {                        // go to normal interval without deepsleep
      long now = millis();                        // count the seconds since the arduino board startet
      if (now - lastMsg > MQTT_INTERVAL*1000) {   
        lastMsg = now;

        char buf[1024];
        toJson().toCharArray(buf, 1024);          // json String to char Array
        client.publish(MQTT_TOPIC, buf);          // send the json to the mq
      }
    } else {
        char buf[1024];
        toJson().toCharArray(buf, 1024);          // json String to char Array
        client.publish(MQTT_TOPIC, buf);          // send the json to the mq
        delay(5);                                 // wait while sending the message
    }
  }



  // -- Web-Server -------------------------------------
  if (WEB_SERVER == 1) {    
    WiFiClient wifiClient = server.available();   // listen for incoming clients
    if (wifiClient) {    
  
      // send a standard http response header
      wifiClient.println("HTTP/1.1 200 OK");
      wifiClient.println("Content-Type: text/html");
      wifiClient.println("Connection: close");
      wifiClient.println();
      wifiClient.println("<!DOCTYPE HTML><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
      wifiClient.println("<meta http-equiv=\"refresh\" content=\"30\"></head>");
      wifiClient.println("<body><div style=\"font-size: 3.5rem;\"><p></p><p>");
      wifiClient.println("<div>Temperatuur: ");
      wifiClient.println(getTemperature());
      wifiClient.println("°C</p></div><p>Luchtvochtigheid: ");
      wifiClient.println(getHumidity());
      wifiClient.println("%</p><p>Water: ");
      wifiClient.println(getWater());
      wifiClient.println("</p></div></div>");
      wifiClient.println(toJson());
      wifiClient.println("</body></html>");     
      // give the web browser time to receive the data
      delay(2);
  
      wifiClient.stop();                          // close the connection
    }
  } else {
    //esp_deep_sleep_enable_timer_wakeup(MQTT_INTERVAL * 1000000);    // Go to deepsleep if the Webserver is offline
    //esp_deep_sleep_start();
  }
}



// MQTT Connect
void mqttconnect() {
  while (!client.connected()) {                   // Try to connect to MQTT
    Serial.print("MQTT connecting...");
    if (client.connect(CLIENT)) {                 // Connect
      Serial.println("connected");
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);                                // Wait 5 seconds before retrying
    }
  }




}
