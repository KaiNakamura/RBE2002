#pragma once

#if defined(ESP8266)
#include <ESP8266WiFi.h>  //ESP8266 Core WiFi Library
#elif defined(ESP32)
#include <WiFi.h>  //ESP32 Core WiFi Library
#else
#include <WiFi.h>
#endif

#include <PubSubClient.h>

bool wifi_reconnect(bool forceReconnect = false);
bool mqtt_reconnect(uint32_t timeout = 0);

//obsolete; just use xyz_reconnect methods; left here for legacy
void setup_wifi(void);
void setup_mqtt(void);
bool reconnect(void);

//void callback(char* topic, byte *payload, unsigned int length);

extern WiFiClient wifiClient;
extern PubSubClient client;
