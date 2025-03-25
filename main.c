//#define DEBUG  // einkommentieren für serielle Ausgabe

#ifdef DEBUG
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "secrets.h"  // enthält WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT

// === GERÄTEKONFIGURATION ===
#define DEVICE_NO 1
#define DEVICE_NAME "sensor" STR(DEVICE_NO)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// === MQTT-TOPICS ===
#define TOPIC_TEMP "sensors/" DEVICE_NAME "/temperature"
#define TOPIC_HUM  "sensors/" DEVICE_NAME "/humidity"

// === INTERVALLKONFIGURATION ===
#define INTERVAL_MINUTES 10
#define SLEEP_DURATION_MS (INTERVAL_MINUTES * 60UL * 1000UL)

// === LED ===
#define LED_PIN LED_BUILTIN  // GPIO2 (D4), aktiv-low
#define BLINK_SUCCESS_MS 200

// === GLOBALE OBJEKTE ===
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Adafruit_BME280 bme;
unsigned long lastSendTime = 0;

// === FUNKTIONEN ===
void blinkSuccess() {
  digitalWrite(LED_PIN, LOW);  // LED an (aktiv-low)
  delay(BLINK_SUCCESS_MS);
  digitalWrite(LED_PIN, HIGH); // LED aus
}

bool connect_wifi() {
  WiFi.forceSleepWake();
  delay(1);  // kurz warten, damit WLAN stabil wird
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  DEBUG_PRINT("Verbinde mit WLAN");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    DEBUG_PRINT(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINTLN("\nWLAN verbunden.");
    return true;
  } else {
    DEBUG_PRINTLN("\nWLAN-Verbindung fehlgeschlagen.");
    return false;
  }
}

bool connect_and_send() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

  DEBUG_PRINT("Verbinde mit MQTT-Broker...");
  if (!mqttClient.connect(DEVICE_NAME)) {
    DEBUG_PRINT(" fehlgeschlagen, rc=");
    DEBUG_PRINTLN(mqttClient.state());
    return false;
  }

  DEBUG_PRINTLN(" erfolgreich.");

  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    DEBUG_PRINTLN("Sensorfehler: Temperatur oder Feuchte ungültig.");
    mqttClient.disconnect();
    return false;
  }

  char tempStr[16];
  char humStr[16];
  dtostrf(temperature, 6, 2, tempStr);
  dtostrf(humidity, 6, 2, humStr);

  mqttClient.publish(TOPIC_TEMP, tempStr, true);
  mqttClient.publish(TOPIC_HUM, humStr, true);

  DEBUG_PRINTF("MQTT gesendet: %s → %s\n", TOPIC_TEMP, tempStr);
  DEBUG_PRINTF("MQTT gesendet: %s → %s\n", TOPIC_HUM, humStr);

  mqttClient.disconnect();
  blinkSuccess();
  return true;
}

void enter_light_sleep() {
  DEBUG_PRINTF("Light Sleep für %d Minuten...\n", INTERVAL_MINUTES);
  WiFi.disconnect(true);        // WLAN trennen, DHCP-Info löschen
  WiFi.forceSleepBegin();       // WLAN-Hardware schlafen legen
  delay(SLEEP_DURATION_MS);     // Light Sleep (CPU läuft weiter, WLAN schläft)
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  delay(100); // serielle Stabilität
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED aus (aktiv-low)

  Wire.begin();  // I2C Pins D1 & D2 (SDA, SCL)

  if (!bme.begin(0x76)) {
    DEBUG_PRINTLN("BME280 nicht gefunden!");
    while (true) delay(1000);  // Stoppe dauerhaft
  }

  if (MDNS.begin("esp8266")) {
    DEBUG_PRINTLN("mDNS gestartet");
  } else {
    DEBUG_PRINTLN("mDNS konnte nicht gestartet werden");
  }

  // Beim Start sofort senden
  lastSendTime = millis() - SLEEP_DURATION_MS;
}

void loop() {
  unsigned long now = millis();
  if (now - lastSendTime >= SLEEP_DURATION_MS) {
    lastSendTime = now;

    if (connect_wifi()) {
      connect_and_send();
    }

    enter_light_sleep();
  }

  // Optional: CPU idle halten
  delay(50);
}
