#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "time.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <queue>
#include <vector>

#define ENABLE_FIRESTORE
#define ENABLE_DATABASE
#define ENABLE_USER_AUTH
#include <FirebaseClient.h>
#include <ArduinoJson.h>

const char* ssid = "Iphoen";
const char* password = "12345678";

#define FIREBASE_PROJECT_ID ""
#define FIREBASE_AUTH "" 

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

// --- KONFIGURACJA PINÓW ---
#define WIFI_STATUS_PIN 4 // Ten pin idzie do PC1 w STM32
#define LED_PIN 2         // Dioda na ESP32

#define RXD2 16
#define TXD2 17
#define UART_BAUD 115200 

WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);

NoAuth no_auth;
FirebaseApp app;
Firestore::Documents Docs;

String deviceId;

// ==========================================
// STRUKTURA KOLEJKI
// ==========================================
struct QueueItem {
    bool isEcg;
    bool isPatch; 
    time_t timestamp;
    String docId;
    String collectionName;
    std::vector<int> ecgValues;
    int batchIndex;
    float value;   
    String mood;    
    String name;    
    String surname; 
};

std::queue<QueueItem> uploadQueue;
unsigned long lastUploadTime = 0;
const int UPLOAD_INTERVAL = 50; 

String lastPayload = "";    
time_t lastEcgTimestamp = 0;   
int ecgBatchIndex = 0;         
String inputBuffer = ""; 

bool wasConnected = false;

void processData(AsyncResult &aResult) { }

uint8_t calcChecksum(const String& json) {
  uint8_t sum = 0;
  for (int i = 0; i < json.length(); i++) {
    sum += json[i];
  }
  return sum;
}

// ... (Funkcje enqueueEcg, enqueueValue, enqueueFacts - BEZ ZMIAN) ...
void enqueueEcg(time_t timestamp, JsonArray ecgData) {
    if (timestamp == lastEcgTimestamp) ecgBatchIndex++;
    else { lastEcgTimestamp = timestamp; ecgBatchIndex = 0; }
    QueueItem item; item.isEcg = true; item.isPatch = false; item.timestamp = timestamp; 
    item.batchIndex = ecgBatchIndex; item.docId = String(timestamp) + "_" + String(ecgBatchIndex) + "_" + deviceId;
    for (JsonVariant v : ecgData) item.ecgValues.push_back(v.as<int>());
    uploadQueue.push(item);
}
void enqueueValue(time_t timestamp, String collection, float val, String mood = "") {
    QueueItem item; item.isEcg = false; item.isPatch = false; item.timestamp = timestamp;
    item.docId = String(timestamp) + "_" + deviceId; item.collectionName = collection; item.value = val; item.mood = mood;
    uploadQueue.push(item);
}
void enqueueFacts(String name, String surname) {
    QueueItem item; item.isEcg = false; item.isPatch = true; item.name = name; item.surname = surname;
    uploadQueue.push(item);
}

void processQueue() {
    if (uploadQueue.empty() || millis() - lastUploadTime < UPLOAD_INTERVAL) return;
    if (WiFi.status() != WL_CONNECTED) return; 

    QueueItem item = uploadQueue.front();
    Document<Values::Value> doc;
    doc.add("deviceId", Values::Value(Values::StringValue(deviceId)));
    if (!item.isPatch) doc.add("timestamp", Values::Value(Values::IntegerValue(item.timestamp)));

    if (item.isEcg) {
        doc.add("batchIndex", Values::Value(Values::IntegerValue(item.batchIndex)));
        if (!item.ecgValues.empty()) {
            Values::Value firstVal((Values::IntegerValue(item.ecgValues[0])));
            Values::ArrayValue arr(firstVal);
            for (size_t i = 1; i < item.ecgValues.size(); i++) arr.add(Values::Value(Values::IntegerValue(item.ecgValues[i])));
            doc.add("values", Values::Value(arr));
        }
        Docs.createDocument(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), "ecg_measurements", item.docId, DocumentMask(), doc, processData, "uTask");
    } else if (item.isPatch) {
        doc.add("name", Values::Value(Values::StringValue(item.name)));
        doc.add("surname", Values::Value(Values::StringValue(item.surname)));
        Docs.patch(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), "facts/" + deviceId, PatchDocumentOptions(DocumentMask("deviceId,name,surname"), DocumentMask(), Precondition()), doc, processData, "fTask");
    } else {
        doc.add("value", Values::Value(Values::DoubleValue(item.value)));
        if (item.mood.length() > 0) doc.add("mood", Values::Value(Values::StringValue(item.mood)));
        Docs.createDocument(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), item.collectionName, item.docId, DocumentMask(), doc, processData, "uTask");
    }
    uploadQueue.pop();
    lastUploadTime = millis();
}

void parseMessage(String msg) {
    msg.trim();
    if (msg.length() == 0) return;

    Serial.print("[RX]: "); Serial.println(msg);

    // A. GET_RTC
    if (msg.indexOf("GET_RTC") >= 0) {
        Serial.println("[RTC] Request detected!");
        delay(50); 
        time_t now = time(NULL);
        String payload = String("{\"rtc\":") + now + "}";
        uint8_t crc = calcChecksum(payload);
        String json = payload + ",\"crc\":" + String(crc) + "}\n";
        Serial2.print(json); 
        Serial2.flush();
        Serial.println("[RTC] Sent time to STM32");
        return;
    }

    // B. DANE JSON
    if (msg.startsWith("{")) {
        int crcIndex = msg.lastIndexOf(",\"crc\":");
        if (crcIndex != -1 && msg.endsWith("}")) {
            String payload = msg.substring(0, crcIndex);
            if (payload.endsWith(",")) payload.remove(payload.length() - 1);

            int receivedCRC = msg.substring(crcIndex + 7, msg.length() - 1).toInt();
            int calculatedCRC = calcChecksum(payload);

            if (receivedCRC == calculatedCRC) {
                // ACK FIRST
                Serial2.print("ACK\n");
                Serial2.flush(); 
                Serial.println("[ACK] Sent");

                if (payload == lastPayload) {
                     Serial.println("[DUPLICATE] Ignored.");
                } else {
                    lastPayload = payload;
                    
                    DynamicJsonDocument doc(4096); 
                    DeserializationError error = deserializeJson(doc, payload);
                    if (!error) {
                        time_t ts = doc["timestamp"] | time(NULL);
                        Serial.println("--- PARSING DATA ---");
                        
                        if (doc.containsKey("ecg")) {
                            Serial.println(" -> Found ECG");
                            enqueueEcg(ts, doc["ecg"].as<JsonArray>());
                        }
                        if (doc.containsKey("object")) {
                            Serial.println(" -> Found Temperature");
                            enqueueValue(ts, "temperature_measurements", doc["object"]);
                        }
                        else if (doc.containsKey("ambient")) {
                            enqueueValue(ts, "temperature_measurements", doc["ambient"]);
                        }
                        
                        if (doc.containsKey("hr")) {
                            Serial.println(" -> Found HeartRate");
                            enqueueValue(ts, "pulse_measurements", doc["hr"]);
                        }
                        else if (doc.containsKey("heartbeat")) {
                            enqueueValue(ts, "pulse_measurements", doc["heartbeat"]);
                        }
                        
                        if (doc.containsKey("spo2")) {
                            Serial.println(" -> Found SpO2");
                            enqueueValue(ts, "spo2_measurements", doc["spo2"]);
                        }
                        
                        if (doc.containsKey("gsr")) {
                            String m = doc.containsKey("mood") ? doc["mood"].as<String>() : "";
                            Serial.printf(" -> Found GSR (Mood: %s)\n", m.c_str());
                            enqueueValue(ts, "gsr_measurements", doc["gsr"], m);
                        }
                        
                        if (doc.containsKey("name")) {
                            Serial.println(" -> Found User Info");
                            enqueueFacts(doc["name"].as<String>(), doc["surname"].as<String>());
                        }
                        Serial.println("--------------------");
                    } else {
                        Serial.print("[JSON ERROR]: "); Serial.println(error.c_str());
                    }
                }
            } else {
                Serial2.print("NACK\n");
                Serial2.flush();
                Serial.printf("[CRC FAIL] Rec: %d, Calc: %d\n", receivedCRC, calculatedCRC);
            }
        }
    }
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
    Serial.begin(115200);
    delay(1000); 

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial2.setRxBufferSize(10240); 
    Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

    // KONFIGURACJA PINU D4 (WYJŚCIE)
    pinMode(WIFI_STATUS_PIN, OUTPUT);
    digitalWrite(WIFI_STATUS_PIN, LOW); // Domyślnie 0 (Brak WiFi)

    WiFi.begin(ssid, password);
    
    int retry = 0;
    while(WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500); Serial.print("."); 
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        retry++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        // !!! WAŻNE: Stan wysoki na starcie !!!
        digitalWrite(WIFI_STATUS_PIN, HIGH);
        wasConnected = true;
        digitalWrite(LED_PIN, HIGH); 
        Serial.println("\nWiFi Connected!");
    } else {
        digitalWrite(WIFI_STATUS_PIN, LOW);
        wasConnected = false;
        Serial.println("\nWiFi Failed!");
    }

    deviceId = WiFi.macAddress();
    deviceId.replace(":", "");
    
    ssl_client.setInsecure();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    if (wasConnected) {
        Serial.print("Syncing Time");
        while (time(NULL) < 1600000000) { delay(100); Serial.print("."); }
        Serial.println("\nTime OK!");
    }

    initializeApp(aClient, app, getAuth(no_auth), processData, "authTask");
    app.getApp<Firestore::Documents>(Docs);

    inputBuffer.reserve(2048); 
    Serial.println("ESP32 READY.");
}

void loop() {
    // ===============================================
    // 1. MONITOROWANIE WIFI (NAPRAWIONE)
    // ===============================================
    bool isConnected = (WiFi.status() == WL_CONNECTED);

    // Wykrywanie zmiany stanu
    if (isConnected != wasConnected) {
        if (!isConnected) {
            // --- UTRACONO POŁĄCZENIE ---
            Serial.println("!!! WIFI LOST !!!");
            
            // !!! TO JEST TA ZMIANA !!!
            digitalWrite(WIFI_STATUS_PIN, LOW); // Wyślij 0V do STM32 (PRZERWANIE)
            
            digitalWrite(LED_PIN, LOW); 
        } 
        else {
            // --- ODZYSKANO POŁĄCZENIE ---
            Serial.println("!!! WIFI RECONNECTED !!!");
            
            // !!! TO JEST TA ZMIANA !!!
            digitalWrite(WIFI_STATUS_PIN, HIGH); // Wyślij 3.3V do STM32 (PRZERWANIE)
            
            digitalWrite(LED_PIN, HIGH); 
        }
        wasConnected = isConnected;
    }

    // Miganie diodą jak brak sieci
    if (!isConnected) {
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
            lastBlink = millis();
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    }

    // ===============================================
    // 2. CZYTANIE UART
    // ===============================================
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') { parseMessage(inputBuffer); inputBuffer = ""; } 
        else { inputBuffer += c; }
    }

    // ===============================================
    // 3. INTERNET
    // ===============================================
    app.loop(); 
    processQueue(); 
}