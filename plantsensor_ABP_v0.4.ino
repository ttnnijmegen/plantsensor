
 /*
*****************************************************************************

* Copyright (c) 2015  Thomas Telkamp and Matthijs Kooijman   
* Copyright (c) 2018  Marvin
* Copyright (c) 2019  Roger & Rob
*
* Permission is hereby granted, free of charge, to anyone
* obtaining a copy of this document and accompanying files,
* to do whatever they want with them without any restriction,
* including, but not limited to, copying, modification and redistribution.
* NO WARRANTY OF ANY KIND IS PROVIDED.
*
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
* Session keys are preconfigured (unlike OTAA, where a DevEUI and
* application key is configured, while the DevAddr and session keys are
* assigned/generated in the over-the-air-activation procedure).
*
* Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
* g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
* violated by this sketch when left running for longer)!
*
* To use this sketch, first register your application and device with
* the things network, to set or generate a DevAddr, NwkSKey and
* AppSKey. Each device should have their own unique values for these
* fields.
*
*******************************************************************************
*
For use of lorawan use the adjusted ESP32 lmic library: https://github.com/matthijskooijman/arduino-lmic
Library used for the DHT sensor is the Adafruit DHT sensor. Therefore also use: https://github.com/adafruit/Adafruit_Sensor

For use of the cayenne LPP add the following code to TheThingsNetwork.h:

******
if (defined(AVR))

include <avr\pgmspace.h>

else

include <pgmspace.h>

endif
********

****************************
Custom Payload decoder (uncomment below and comment the LPP payload)

function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var water=(bytes[0]<<8)|bytes[1];
  var temperature=(bytes[2]<<8)|bytes[3];
  var humidity=(bytes[4]<<8)|bytes[5];
  var light=(bytes[6]<<8)|bytes[7];
  
  return{
    water:water,
    temperature:temperature/100,
    humidity:humidity/10,
    light:light
    }
  }
****************************
*/

#define LED_PIN 2

//int LED_BUILTIN = 2;


// temp board read
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();




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



// Client variables 
long lastMsg = 0;

// lora keys

// LoRaWAN NwkSKey, network session key

static u1_t NWKSKEY[16] = <YOUR NETWORK SESSION KEY IN MSB FORMAT>;

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static u1_t APPSKEY[16] = YOUR APPLICATION SESSION KEY IN MSB FORMAT>;

// LoRaWAN end-device address (DevAddr)
static u4_t DEVADDR = 0x<YOUR DEVICE ADDRESS> ; // <-- Change this address for every node!


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping for HiGrow
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {2, 13, 15},
};


// cayenne
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
  //Serial.print("start do send");
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        //LPP code (comment if use of custom payload
// *********************************************************************
        
        lpp.reset();

        uint8_t water = getWater();
        uint8_t temp = getTemperature();
        uint8_t hum = getHumidity();
        uint8_t HallRead = hallRead();
        uint8_t BoardTemperature = (temprature_sens_read()- 32) / 1.8;
        
        lpp.addTemperature(0, temp);
        lpp.addRelativeHumidity(1, hum);
        lpp.addBarometricPressure(2, water);
        lpp.addAnalogOutput(3, HallRead);
        lpp.addTemperature(4, BoardTemperature);

        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Packet queued"));
// *********************************************************************

//// *********************************************************************
        // Custom payload        
//        byte payload[8];
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
//        LMIC_setTxData2(1, payload, 8, 0);
//        Serial.println(F("Packet queued"));
//// *********************************************************************

    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// Build JSON
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
  dht.begin();                                    // initialize the DHT sensor
  // define blue light
  pinMode(LED_PIN, OUTPUT);
  digitalWrite (LED_PIN, LOW);
  
 
  //Initialize serial and wait for port to open:
  Serial.begin(115200);

    // LMIC init
    //Serial.print("os init");
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.

    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);


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
    
    // Start job
    //Serial.print("do send job");
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
      //Serial.print("loop once");
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



