#include <WiFi.h>
#include "time.h"

// ---- Ustawienia Wi-Fi ----
const char* ssid = "Iphoen";
const char* password = "12345678";

// ---- NTP (czas internetowy) ----
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;      // dla czasu UTC
const int daylightOffset_sec = 0;  // brak DST

// ---- UART (STM32 <-> ESP32) ----
#define RXD2 16  // STM32 TX → ESP32 RX
#define TXD2 17  // nieużywany, ale wymagany

uint8_t calcChecksum(const String& json) {
  uint8_t sum = 0;
  for (int i = 0; i < json.length(); i++) {
    sum += json[i];
  }
  return sum;
}

void sendWithAck(String json) {
  const int MAX_RETRIES = 3;
  for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
    Serial2.print(json);  // wyślij JSON z checksum
    Serial.print("[ESP32] Sent: ");
    Serial.println(json);

    unsigned long start = millis();
    String ack = "";

    while (millis() - start < 500) {
      if (Serial2.available()) {
        ack = Serial2.readStringUntil('\n');
        ack.trim();
        break;
      }
    }

    if (ack == "ACK") {
  Serial.println("[ESP32] ACK received");
  return;
} else if (ack == "NACK") {
  Serial.println("[ESP32] NACK received — retrying...");
} else {
  Serial.println("[ESP32] No response or unknown reply — retrying...");
}
  }

  Serial.println("[ESP32] Retries exhausted — failed to confirm.");
}


void setup() {
  Serial.begin(9600);   // debug
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("ESP32 booting...");

  // ---- Łączenie z Wi-Fi ----
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // ---- Konfiguracja NTP ----
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // ---- Pobranie czasu ----
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  } else {
    time_t now = time(NULL);
    Serial.printf("Sending time to STM32: %lu\n", now);
    String json = String("{\"rtc\":") + now + "}";
    uint8_t crc = calcChecksum(json);
    json += ",\"crc\":" + String(crc) + "}";  // bez \n w treści JSON-a
json += "\n"; 
    sendWithAck(json);
  }

  delay(1000);
  Serial.println("ESP32 UART Receiver ready.");
}

void loop() {
if (Serial2.available()) {
  String msg = Serial2.readStringUntil('\n');
  msg.trim();

  Serial.print("[ESP32 DEBUG] Received: ");
  Serial.println(msg);

  if (msg == "GET_RTC") {
    time_t now = time(NULL);
    String payload = String("{\"rtc\":") + now + "}";
    uint8_t crc = calcChecksum(payload);
    String json = payload + ",\"crc\":" + String(crc) + "}\n";
    sendWithAck(json);
  } else if (msg.startsWith("{")) {
  int crcIndex = msg.lastIndexOf(",\"crc\":");
  if (crcIndex != -1 && msg.endsWith("}")) {
    String payload = msg.substring(0, crcIndex);
    if (payload.endsWith(",")) {
      payload.remove(payload.length() - 1);
    }

    int receivedCRC = msg.substring(crcIndex + 7, msg.length() - 1).toInt();
    int calculatedCRC = calcChecksum(payload);


    if (receivedCRC == calculatedCRC) {
      Serial.println("[ESP32] Sending ACK");
      Serial2.println("ACK");
    } else {
      Serial.println("[ESP32] Sending NACK");
      Serial2.println("NACK");
    }
  }

  }
}

  delay(10);  // nie blokuj CPU
}
