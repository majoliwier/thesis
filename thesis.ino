#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "time.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <queue>
#include <vector>

// --- NOWE BIBLIOTEKI ---
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2901.h>

#define ENABLE_FIRESTORE
#define ENABLE_DATABASE
#define ENABLE_USER_AUTH
#include <FirebaseClient.h>
#include <ArduinoJson.h>

// --- KONFIGURACJA PINÓW ---
#define WIFI_STATUS_PIN 4 
#define LED_PIN 2        
#define RXD2 16
#define TXD2 17
#define UART_BAUD 115200 
#define BUTTON_PIN 0      // Przycisk BOOT na ESP32

// --- ZMIENNE WIFI (Dynamiczne) ---
String wifi_ssid = "";
String wifi_password = "";

// --- FIREBASE CONFIG ---
#define FIREBASE_PROJECT_ID ""
#define FIREBASE_AUTH "" 

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);

NoAuth no_auth;
FirebaseApp app;
Firestore::Documents Docs;

String deviceId;

// --- PREFERENCES & BLE ---
Preferences preferences;
bool bleMode = false;
unsigned long buttonPressTime = 0;
bool buttonHeld = false;

// UUID dla usługi BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_SSID_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_PASS_UUID      "8899aabb-ccdd-eeff-0011-223344556677"

BLEServer* pServer = NULL;
BLECharacteristic* pCharSSID = NULL;
BLECharacteristic* pCharPass = NULL;

// ==========================================
// 1. ZMIENNE GLOBALNE DO BUFOROWANIA
// ==========================================
String rawConfigPayload = ""; 
bool payloadReady = false;    
bool fetchError = false;      

// ==========================================
// 2. STRUKTURA KONFIGURACJI
// ==========================================
struct UserConfig {
    String name = "Jan";
    String surname = "Kowalski";
    float temp_min = 35.0;
    float temp_max = 37.5;
    int spo2_min = 90;
    int hr_min = 40;
    int hr_max = 120;
    int gsr_max1 = 600;
    int gsr_max2 = 1000;
    bool loaded = false; 
} currentConfig;

// ==========================================
// STRUKTURA KOLEJKI
// ==========================================
struct QueueItem {
    bool isEcg;
    bool isPatch; 
    bool isAlarm;
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

// --- PROTOTYPY ---
void syncConfigBlocking(); 
void parseConfigSafe(); 
void onCreateResult(AsyncResult &aResult);
void onFetchResult(AsyncResult &aResult);
void printResult(AsyncResult &aResult);
void setupBLE();

// ==========================================
// CALLBACKI BLE (Obsługa zapisu)
// ==========================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("[BLE] Device connected");
    };
    void onDisconnect(BLEServer* pServer) {
      Serial.println("[BLE] Device disconnected");
      pServer->getAdvertising()->start(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      // POPRAWKA: Używamy String zamiast std::string
      String value = pCharacteristic->getValue();
      
      if (value.length() > 0) {
        if (pCharacteristic->getUUID().toString() == CHAR_SSID_UUID) {
            Serial.println("[BLE] Received SSID: " + value);
            preferences.begin("net_config", false); 
            preferences.putString("ssid", value);
            preferences.end();
        } 
        else if (pCharacteristic->getUUID().toString() == CHAR_PASS_UUID) {
            Serial.println("[BLE] Received Password. Saving and Restarting...");
            preferences.begin("net_config", false);
            preferences.putString("pass", value);
            preferences.end();
            
            delay(1000);
            ESP.restart(); 
        }
      }
    }
};


// ==========================================
// FUNKCJE FIREBASE
// ==========================================
void printResult(AsyncResult &aResult) {
    if (aResult.isError()) {
        Serial.printf("[FIREBASE ERR] %s\n", aResult.error().message().c_str());
    }
}

void syncConfigBlocking() {
    Serial.println("\n[SYNC] --- STARTING SAFE SYNC ---");
    currentConfig.loaded = false;
    bool exists = false;

    Document<Values::Value> doc;
    doc.add("name", Values::Value(Values::StringValue(currentConfig.name)));
    doc.add("surname", Values::Value(Values::StringValue(currentConfig.surname)));
    doc.add("temp_min", Values::Value(Values::DoubleValue(currentConfig.temp_min)));
    doc.add("temp_max", Values::Value(Values::DoubleValue(currentConfig.temp_max)));
    doc.add("spo2_min", Values::Value(Values::IntegerValue(currentConfig.spo2_min)));
    doc.add("hr_min", Values::Value(Values::IntegerValue(currentConfig.hr_min)));
    doc.add("hr_max", Values::Value(Values::IntegerValue(currentConfig.hr_max)));
    doc.add("gsr_max1", Values::Value(Values::IntegerValue(currentConfig.gsr_max1)));
    doc.add("gsr_max2", Values::Value(Values::IntegerValue(currentConfig.gsr_max2)));

    Docs.createDocument(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), "user_config", deviceId, DocumentMask(), doc, onCreateResult, "syncTask");

    unsigned long t = millis();
    while (!currentConfig.loaded && !fetchError && millis() - t < 10000) {
        app.loop();
        if (rawConfigPayload == "EXISTS") { exists = true; break; } 
        delay(10);
    }

    if (currentConfig.loaded) {
        Serial.println("[SYNC] Document created. Using defaults.");
        return; 
    }

    if (exists) {
        Serial.println("[SYNC] Document exists. Fetching raw data...");
        rawConfigPayload = "";
        payloadReady = false;
        fetchError = false;
        Docs.get(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), "user_config/" + deviceId, GetDocumentOptions(), onFetchResult, "syncTask");
        t = millis();
        while (!payloadReady && !fetchError && millis() - t < 10000) {
            app.loop();
            delay(10);
        }
        if (payloadReady) {
            Serial.println("[SYNC] Raw data captured. Parsing safely...");
            parseConfigSafe(); 
        } else {
            Serial.println("[SYNC] Fetch Timeout or Error.");
            currentConfig.loaded = true; 
        }
    } else {
        currentConfig.loaded = true;
    }
}

void onCreateResult(AsyncResult &aResult) {
    if (aResult.available()) {
        currentConfig.loaded = true; 
    } 
    else if (aResult.isError()) {
        if (aResult.error().code() == 409) {
            rawConfigPayload = "EXISTS"; 
        }
    }
}

void onFetchResult(AsyncResult &aResult) {
    if (aResult.isError()) {
        Serial.printf("[SYNC ERR] %s\n", aResult.error().message().c_str());
        fetchError = true;
        return;
    }
    if (aResult.available()) {
        rawConfigPayload = aResult.payload();
        payloadReady = true;
    }
}

void parseConfigSafe() {
    DynamicJsonDocument doc(4096); 
    DeserializationError err = deserializeJson(doc, rawConfigPayload);
    rawConfigPayload = ""; 

    if (!err) {
        JsonObject fields = doc["fields"];
        if (!fields.isNull()) {
            if (fields.containsKey("temp_min")) {
                JsonObject f = fields["temp_min"];
                if (f.containsKey("doubleValue")) currentConfig.temp_min = f["doubleValue"].as<float>();
                else if (f.containsKey("integerValue")) currentConfig.temp_min = f["integerValue"].as<float>();
            }
            if (fields.containsKey("temp_max")) {
                JsonObject f = fields["temp_max"];
                if (f.containsKey("doubleValue")) currentConfig.temp_max = f["doubleValue"].as<float>();
                else if (f.containsKey("integerValue")) currentConfig.temp_max = f["integerValue"].as<float>();
            }
            if (fields.containsKey("spo2_min")) {
                JsonObject f = fields["spo2_min"];
                if (f.containsKey("integerValue")) currentConfig.spo2_min = f["integerValue"].as<int>();
                else if (f.containsKey("doubleValue")) currentConfig.spo2_min = f["doubleValue"].as<int>();
            }
            if (fields.containsKey("hr_min")) {
                JsonObject f = fields["hr_min"];
                if (f.containsKey("integerValue")) currentConfig.hr_min = f["integerValue"].as<int>();
                else if (f.containsKey("doubleValue")) currentConfig.hr_min = f["doubleValue"].as<int>();
            }
            if (fields.containsKey("hr_max")) {
                JsonObject f = fields["hr_max"];
                if (f.containsKey("integerValue")) currentConfig.hr_max = f["integerValue"].as<int>();
                else if (f.containsKey("doubleValue")) currentConfig.hr_max = f["doubleValue"].as<int>();
            }
            if (fields.containsKey("gsr_max1")) {
                JsonObject f = fields["gsr_max1"];
                if (f.containsKey("integerValue")) currentConfig.gsr_max1 = f["integerValue"].as<int>();
                else if (f.containsKey("doubleValue")) currentConfig.gsr_max1 = f["doubleValue"].as<int>();
            }
            if (fields.containsKey("gsr_max2")) {
                JsonObject f = fields["gsr_max2"];
                if (f.containsKey("integerValue")) currentConfig.gsr_max2 = f["integerValue"].as<int>();
                else if (f.containsKey("doubleValue")) currentConfig.gsr_max2 = f["doubleValue"].as<int>();
            }
            if (fields.containsKey("name")) currentConfig.name = fields["name"]["stringValue"].as<String>();
            if (fields.containsKey("surname")) currentConfig.surname = fields["surname"]["stringValue"].as<String>();

            Serial.println("\n--------------------------------------------------");
            Serial.println("[SYNC] FULL CONFIGURATION LOADED FROM FIREBASE:");
            Serial.printf("  Name:      %s\n", currentConfig.name.c_str());
            Serial.printf("  Surname:   %s\n", currentConfig.surname.c_str());
            Serial.println("--------------------------------------------------\n");
        }
    }
    currentConfig.loaded = true;
}

// ==========================================
// RESZTA PROGRAMU
// ==========================================

uint8_t calcChecksum(const String& json) {
    uint8_t sum = 0;
    for (int i = 0; i < json.length(); i++) sum += json[i];
    return sum;
}

void enqueueEcg(time_t timestamp, JsonArray ecgData) {
    if (timestamp == lastEcgTimestamp) ecgBatchIndex++;
    else { lastEcgTimestamp = timestamp; ecgBatchIndex = 0; }
    QueueItem item; item.isEcg = true; item.isPatch = false; item.timestamp = timestamp; 
    item.batchIndex = ecgBatchIndex; item.docId = String(timestamp) + "_" + String(ecgBatchIndex) + "_" + deviceId;
    for (JsonVariant v : ecgData) item.ecgValues.push_back(v.as<int>());
    uploadQueue.push(item);
}
void enqueueValue(time_t timestamp, String collection, float val, bool alarm, String mood = "") {
    QueueItem item; item.isEcg = false; item.isPatch = false; item.isAlarm = alarm;
    item.timestamp = timestamp; item.docId = String(timestamp) + "_" + deviceId; 
    item.collectionName = collection; item.value = val; item.mood = mood;
    uploadQueue.push(item);
}

void processQueue() {
    if (uploadQueue.empty() || millis() - lastUploadTime < UPLOAD_INTERVAL) return;
    if (WiFi.status() != WL_CONNECTED) return; 

    QueueItem item = uploadQueue.front();
    Document<Values::Value> doc;
    doc.add("deviceId", Values::Value(Values::StringValue(deviceId)));
    
    if (!item.isPatch) {
        doc.add("timestamp", Values::Value(Values::IntegerValue(item.timestamp)));
        doc.add("alarm", Values::Value(Values::BooleanValue(item.isAlarm))); 
    }

    if (item.isEcg) {
        doc.add("batchIndex", Values::Value(Values::IntegerValue(item.batchIndex)));
        if (!item.ecgValues.empty()) {
            Values::Value firstVal((Values::IntegerValue(item.ecgValues[0])));
            Values::ArrayValue arr(firstVal);
            for (size_t i = 1; i < item.ecgValues.size(); i++) arr.add(Values::Value(Values::IntegerValue(item.ecgValues[i])));
            doc.add("values", Values::Value(arr));
        }
        Docs.createDocument(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), "ecg_measurements", item.docId, DocumentMask(), doc, printResult, "uTask");
    } else if (!item.isPatch) {
        doc.add("value", Values::Value(Values::DoubleValue(item.value)));
        if (item.mood.length() > 0) doc.add("mood", Values::Value(Values::StringValue(item.mood)));
        Docs.createDocument(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), item.collectionName, item.docId, DocumentMask(), doc, printResult, "uTask");
    }
    uploadQueue.pop();
    lastUploadTime = millis();
}

void parseMessage(String msg) {
    msg.trim();
    if (msg.length() == 0) return;

    if (msg.indexOf("GET_TIME") >= 0) {
        delay(20); 
        time_t now = time(NULL);
        String json = "{\"rtc\":" + String(now) + "}";
        Serial2.print(json + "\n");
        return;
    }

    if (msg.indexOf("GET_CONFIG") >= 0) {
        if (!currentConfig.loaded) {
            Serial2.print("{\"status\":\"wait\"}\n"); 
            return;
        }
        delay(20); 
        DynamicJsonDocument doc(512);
        doc["t_min"] = currentConfig.temp_min;
        doc["t_max"] = currentConfig.temp_max;
        doc["sp_min"] = currentConfig.spo2_min;
        doc["hr_min"] = currentConfig.hr_min;
        doc["hr_max"] = currentConfig.hr_max;
        doc["gsr1"]   = currentConfig.gsr_max1;
        doc["gsr2"]   = currentConfig.gsr_max2;
        String payload;
        serializeJson(doc, payload);
        Serial2.print(payload + "\n");
        Serial.println("[TX] Config sent");
        return;
    }

    if (msg.startsWith("{")) {
        int crcIndex = msg.lastIndexOf(",\"crc\":");
        if (crcIndex != -1 && msg.endsWith("}")) {
            String payload = msg.substring(0, crcIndex);
            if (payload.endsWith(",")) payload.remove(payload.length() - 1);

            int receivedCRC = msg.substring(crcIndex + 7, msg.length() - 1).toInt();
            int calculatedCRC = calcChecksum(payload);

            if (receivedCRC == calculatedCRC) {
                Serial2.print("ACK\n");
                if (payload != lastPayload) {
                    lastPayload = payload;
                    DynamicJsonDocument doc(2048); 
                    DeserializationError error = deserializeJson(doc, payload);
                    if (!error) {
                        time_t ts = doc["timestamp"] | time(NULL);
                        bool isAlarm = doc["alarm"] | 0; 
                        
                        if (doc.containsKey("ecg")) enqueueEcg(ts, doc["ecg"].as<JsonArray>());
                        if (doc.containsKey("object")) enqueueValue(ts, "temperature_measurements", doc["object"], isAlarm);
                        else if (doc.containsKey("ambient")) enqueueValue(ts, "temperature_measurements", doc["ambient"], isAlarm);
                        if (doc.containsKey("hr")) enqueueValue(ts, "pulse_measurements", doc["hr"], isAlarm);
                        else if (doc.containsKey("heartbeat")) enqueueValue(ts, "pulse_measurements", doc["heartbeat"], isAlarm);
                        if (doc.containsKey("spo2")) enqueueValue(ts, "spo2_measurements", doc["spo2"], isAlarm);
                        if (doc.containsKey("gsr")) {
                            String m = doc.containsKey("mood") ? doc["mood"].as<String>() : "";
                            enqueueValue(ts, "gsr_measurements", doc["gsr"], isAlarm, m);
                        }
                    } 
                }
            } else {
                Serial2.print("NACK\n");
            }
        }
    }
}

// ==========================================
// SETUP BLE (Z ROZŁĄCZANIEM WIFI + FIX CRASHY)
// ==========================================
void setupBLE() {
    if (bleMode) return; 
    
    // --- NOWOŚĆ: ROZŁĄCZ WIFI ---
    Serial.println("\n[BLE] Disconnecting WiFi for configuration...");
    WiFi.disconnect(true);   // Rozłącz
    WiFi.mode(WIFI_OFF);     // Wyłącz radio WiFi
    digitalWrite(WIFI_STATUS_PIN, LOW); 
    wasConnected = false;    
    
    // !!! WAŻNE !!! 
    // Dajemy chwilę procesorowi na wyłączenie radia WiFi zanim włączymy BLE.
    // To zapobiega błędom "Guru Meditation Error" i "hci_hal" widocznym w logach.
    delay(500); 
    // ----------------------------

    bleMode = true;
    Serial.println("[BLE] STARTING BLE SERVER...");
    
    // Sygnalizacja LED
    for(int i=0; i<5; i++) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
    digitalWrite(LED_PIN, HIGH); 

    BLEDevice::init("ESP32_Medical_Config");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // SSID
    pCharSSID = pService->createCharacteristic(CHAR_SSID_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    pCharSSID->setCallbacks(new MyCallbacks());
    pCharSSID->setValue("Wpisz SSID");
    BLE2901 *pDescSSID = new BLE2901();
    pDescSSID->setDescription("Nazwa Sieci WiFi");
    pCharSSID->addDescriptor(pDescSSID);

    // HASŁO
    pCharPass = pService->createCharacteristic(CHAR_PASS_UUID, BLECharacteristic::PROPERTY_WRITE);
    pCharPass->setCallbacks(new MyCallbacks());
    BLE2901 *pDescPass = new BLE2901();
    pDescPass->setDescription("Haslo WiFi");
    pCharPass->addDescriptor(pDescPass);

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Ready. Search for 'ESP32_Medical_Config'");
}

// ==========================================
// SETUP (Z FIXEM NIESKOŃCZONEJ PĘTLI)
// ==========================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
    Serial.begin(115200);
    delay(1000); 

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial2.setRxBufferSize(10240); 
    Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

    pinMode(WIFI_STATUS_PIN, OUTPUT);
    digitalWrite(WIFI_STATUS_PIN, LOW); 

    // --- WCZYTANIE DANYCH WIFI Z PAMIĘCI ---
    preferences.begin("net_config", true);
    wifi_ssid = preferences.getString("ssid", "");
    wifi_password = preferences.getString("pass", "");
    preferences.end();

    if (wifi_ssid == "") {
        Serial.println("No WiFi credentials saved! Hold button to Config.");
        wifi_ssid = "Iphoen"; 
        wifi_password = "default_pass_maybe";
    } else {
        Serial.println("Read WiFi Creds: " + wifi_ssid);
    }

    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
    
    // Próba połączenia (z możliwością wejścia w BLE przyciskiem)
    int retry = 0;
    while(WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500); Serial.print("."); 
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        
        if(digitalRead(BUTTON_PIN) == LOW) {
             if (!buttonHeld) { buttonPressTime = millis(); buttonHeld = true; }
             if (millis() - buttonPressTime > 3000) { setupBLE(); break; } 
        } else {
             buttonHeld = false;
        }
        retry++;
    }
    
    // Jeśli nie weszliśmy w tryb BLE, kontynuujemy normalny rozruch
    if (!bleMode) {
        if (WiFi.status() == WL_CONNECTED) {
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
        ssl_client.setTimeout(10000); 

        // Konfiguracja czasu
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

        // --- POPRAWIONA PĘTLA CZASU ---
        if (wasConnected) {
            Serial.print("Syncing Time");
            unsigned long timeStart = millis();
            
            // Czekamy na czas, ALE przerywamy jeśli:
            // 1. Minęło 15 sekund (Timeout)
            // 2. LUB straciliśmy połączenie WiFi
            while (time(NULL) < 1600000000 && (millis() - timeStart < 15000) && WiFi.status() == WL_CONNECTED) { 
                delay(100); 
                Serial.print("."); 
            }
            
            if (time(NULL) > 1600000000) {
                Serial.println("\nTime OK!");
            } else {
                Serial.println("\nTime Sync Failed (Timeout or WiFi Lost) - Proceeding anyway...");
            }
        }

        initializeApp(aClient, app, getAuth(no_auth), printResult, "authTask");
        app.getApp<Firestore::Documents>(Docs);

        // Synchronizacja tylko jeśli WiFi nadal działa
        if (WiFi.status() == WL_CONNECTED) {
            syncConfigBlocking();
        } else {
            Serial.println("[SYNC] Skipped (No WiFi)");
            currentConfig.loaded = true; // Fallback, żeby program ruszył dalej
        }
    }

    inputBuffer.reserve(2048); 
    Serial.println("ESP32 READY (Loop Started).");
}

void loop() {
    // --- OBSŁUGA PRZYCISKU (AKTYWACJA BLE) ---
    if (digitalRead(BUTTON_PIN) == LOW) { 
        if (!buttonHeld) {
            buttonPressTime = millis();
            buttonHeld = true;
            Serial.println("Button Pressed...");
        } else {
            if (!bleMode && (millis() - buttonPressTime > 3000)) {
                setupBLE(); // Włącz Bluetooth
            }
        }
    } else {
        buttonHeld = false;
    }

    // W trybie BLE ignorujemy resztę logiki
    if (bleMode) {
        delay(100);
        return; 
    }

    app.loop(); 

    bool isConnected = (WiFi.status() == WL_CONNECTED);

    if (isConnected != wasConnected) {
        if (!isConnected) {
            Serial.println("!!! WIFI LOST !!!");
            digitalWrite(WIFI_STATUS_PIN, LOW); 
            digitalWrite(LED_PIN, LOW); 
        } else {
            Serial.println("!!! WIFI RECONNECTED !!!");
            digitalWrite(WIFI_STATUS_PIN, HIGH); 
            digitalWrite(LED_PIN, HIGH); 
        }
        wasConnected = isConnected;
    }

    if (!isConnected) {
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
            lastBlink = millis();
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    }

    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') { parseMessage(inputBuffer); inputBuffer = ""; } 
        else { inputBuffer += c; }
    }

    processQueue(); 
    delay(1); 
}