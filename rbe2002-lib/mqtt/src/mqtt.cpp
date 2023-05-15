#include <mqtt.h>
#include <wifi_credentials.h>

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const uint32_t KEEP_ALIVE_INTERVAL = 20;

uint32_t lastWiFiCxnAttempt = 0;
uint32_t wifiCxnRetryInterval = 5000;
bool connecting = false;

bool wifi_reconnect(bool forceReconnect) {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWiFiCxnAttempt > wifiCxnRetryInterval ||
        forceReconnect) {
      Serial.print("\nMy MAC address is ");
      Serial.print(WiFi.macAddress());

      Serial.print("\nConnecting to ");
      Serial.print(ssid);

      lastWiFiCxnAttempt = millis();
      WiFi.begin(ssid, password);

      connecting = true;
    }

    static uint32_t lastDot = 0;
    if (millis() - lastDot > 500) {
      lastDot = millis();
      Serial.print('.');
    }
  }

  // use separate if for when the above connects
  if (WiFi.status() == WL_CONNECTED) {
    // if we've trying to connect, let us know we succeeded
    if (connecting) {
      Serial.println("Connected with IP address: ");
      Serial.println(WiFi.localIP());

      connecting = false;
    }

    return true;
  }

  return false;
}

uint32_t lastCxnAttempt = 0;
uint32_t cxnRetryInterval = 1500;

bool mqtt_reconnect(uint32_t timeout) {
  bool wifi_cxn = wifi_reconnect();

  if (!wifi_cxn) {
    if (timeout)  //if timeout is not zero, keep trying to reconnect for the timeout period
    {
      uint32_t startTime = millis();
      while (millis() - startTime < timeout) {
        wifi_cxn = wifi_reconnect();
        if (wifi_cxn) {
          break;
        }
      }
    }
  }

  if (!wifi_cxn) {
    return false;
  }

  //try to reconnect once
  if (!client.connected()) {
    if (millis() - lastCxnAttempt > cxnRetryInterval) {
      lastCxnAttempt = millis();

      client.setServer(mqtt_server, mqtt_port);
      client.setKeepAlive(KEEP_ALIVE_INTERVAL);

      Serial.println("MQTT cxn...");

      // Create a random client ID
      String clientId = "ESP32Client-";
      clientId += String(WiFi.macAddress());

      Serial.print("Connecting as: ");
      Serial.println(clientId);

      // Attempt to connect
      if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("Connected to broker");
        return true;
      }

      else {
        Serial.print("Failed, Error = ");
        Serial.print(client.state());
        Serial.println("; will try again");

        return false;
      }
    }

    return false;
  }

  return true;
}

void setup_mqtt(void) {
  wifi_reconnect(true);
  mqtt_reconnect(5000);
}

/** You can use this callback to test connectivity, but it only prints
 * messages -- you'll have to customize it for your purposes (which is
 * better done in your application -- see the examples)
 * */
// void callback(char* topic, byte *payload, unsigned int length)
// {
//     Serial.println(topic);
//     Serial.print(':');
//     Serial.write(payload, length);
//     Serial.println();
// }

// obsolete calls; best to use xyz_reconnect() directly
void setup_wifi(void) {
  mqtt_reconnect();
}
bool reconnect(void) {
  return mqtt_reconnect();
}
