#include <Arduino.h>
#include <core_esp8266_i2s.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h> // Tambahkan SSL
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include <WiFiManager.h>      // Tambahkan WiFiManager

// Forward Declaration
void publishTelemetry();
void callback(char* topic, byte* payload, unsigned int length);

unsigned long lastCommandTime = 0;
bool shouldPublishTelemetry = false;

// ---------------------------------------------------------------------------
// KONFIGURASI MQTT (Detail akan diisi lewat WiFiManager atau hardcode awal)
// ---------------------------------------------------------------------------
const char* mqtt_server = "q3c13c67.ala.asia-southeast1.emqxsl.com";
int mqtt_port = 8883;
const char* mqtt_user = "ASN-Lab";
const char* mqtt_pass = "amphib1an";

WiFiClientSecure espClient;   // Gunakan Secure Client
PubSubClient client(espClient);

const char* DEVICE_CODE     = "TUKIR-GARAGE-01";
const char* TOPIC_TELEMETRY = "tukir/TUKIR-GARAGE-01/telemetry";
const char* TOPIC_STATUS    = "tukir/TUKIR-GARAGE-01/status";
const char* TOPIC_CONTROL   = "tukir/TUKIR-GARAGE-01/command";

// ---------------------------------------------------------------------------
// PIN MAPPING (FIXED: Pindah Servo ke D5 untuk cegah konflik I2S GPIO2)
// ---------------------------------------------------------------------------
const int TRIG_PIN    = D1; // GPIO 5  (Trigger Bersama)
const int ECHO_REAR   = D2; // GPIO 4  (Echo Belakang)
const int ECHO_SIDE   = D6; // GPIO 12 (Echo Samping)

const int SERVO_PIN   = D5; // GPIO 14 (Dipindah dari D4/GPIO2 agar I2S lancar)

const int LED_GREEN   = D0; // GPIO 16 (Active HIGH)
const int LED_YELLOW  = D7; // GPIO 13 (Active HIGH)
const int LED_RED     = D3; // GPIO 0  (Active LOW)

// ---------------------------------------------------------------------------
// PERANGKAT & STATE SYSTEM
// ---------------------------------------------------------------------------
Servo garageServo;

enum SystemState {
  STATE_CLOSED,
  STATE_OPENING_SCAN,
  STATE_PARKING_IN,
  STATE_EXITING,
  STATE_CLOSING
};

SystemState currentState = STATE_CLOSED;
String garageStatusStr  = "CLOSED";
String systemModeStr    = "STANDBY";

float distRear = 0.0;
float distSide = 0.0;
float fuzzyRiskScore = 0.0;
String fuzzyStatusStr = "AMAN";

unsigned long lastTelemetryTime = 0;
unsigned long autoCloseStartTime = 0;
unsigned long lastAudioTime = 0;
bool isAutoClosingTimerActive = false;
int exitValidCounter = 0;

// ---------------------------------------------------------------------------
// ENGINE FUZZY LOGIC TSUKAMOTO (MINIATUR 18.5 cm x 11 cm)
// ---------------------------------------------------------------------------
float mfSangatDekatRear(float x) {
  if (x <= 3.0) return 1.0;
  if (x >= 6.0) return 0.0;
  return (6.0 - x) / 3.0;
}
float mfSedangRear(float x) {
  if (x <= 4.0 || x >= 12.0) return 0.0;
  if (x <= 8.0) return (x - 4.0) / 4.0;
  return (12.0 - x) / 4.0;
}
float mfJauhRear(float x) {
  if (x <= 9.0) return 0.0;
  if (x >= 14.0) return 1.0;
  return (x - 9.0) / 5.0;
}

float mfMiringKiri(float x) {
  if (x <= 3.5) return 1.0;
  if (x >= 5.5) return 0.0;
  return (5.5 - x) / 2.0;
}
float mfIdeal(float x) {
  if (x <= 3.0 || x >= 7.5) return 0.0;
  if (x <= 5.5) return (x - 3.0) / 2.5;
  return (7.5 - x) / 2.0;
}
float mfMiringKanan(float x) {
  if (x <= 5.5) return 0.0;
  if (x >= 7.0) return 1.0;
  return (x - 5.5) / 1.5;
}

void computeFuzzyLogic(float rDist, float sDist) {
  float clampedRear = constrain(rDist, 0.0f, 15.0f);
  float clampedSide = constrain(sDist, 0.0f, 8.0f);

  float rSD = mfSangatDekatRear(clampedRear);
  float rS  = mfSedangRear(clampedRear);
  float rJ  = mfJauhRear(clampedRear);

  float sMKi = mfMiringKiri(clampedSide);
  float sI   = mfIdeal(clampedSide);
  float sMKa = mfMiringKanan(clampedSide);

  float rulesAlpha[9] = {
    min(rSD, sMKi), min(rSD, sI), min(rSD, sMKa),
    min(rS,  sMKi), min(rS,  sI), min(rS,  sMKa),
    min(rJ,  sMKi), min(rJ,  sI), min(rJ,  sMKa)
  };
  float rulesZ[9] = {95, 85, 95, 60, 30, 60, 45, 10, 45};

  float num = 0, den = 0;
  for (int i = 0; i < 9; i++) {
    if (rulesAlpha[i] > 0) {
      num += rulesAlpha[i] * rulesZ[i];
      den += rulesAlpha[i];
    }
  }

  fuzzyRiskScore = (den > 0) ? (num / den) : 0.0;

  if (fuzzyRiskScore >= 65.0) fuzzyStatusStr = "BAHAYA";
  else if (fuzzyRiskScore >= 30.0) fuzzyStatusStr = "WASPADA";
  else fuzzyStatusStr = "AMAN";
}

// ---------------------------------------------------------------------------
// AUDIO DIGITAL I2S
// ---------------------------------------------------------------------------
void playI2STone(int frequencyHz, int durationMs) {
  // Fitur Audio dinonaktifkan untuk stabilitas sistem PWM (Servo/LED)
  return;
}

// ---------------------------------------------------------------------------
// SENSOR ULTRASONIK
// ---------------------------------------------------------------------------
float readUltrasonic(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); 
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

// ---------------------------------------------------------------------------
// KONTROL PINTU & CALLBACK MQTT
// ---------------------------------------------------------------------------
void openGarage() {
  Serial.println("[ACTION] Opening Garage Servo to 90 degrees...");
  garageServo.write(90);
  garageStatusStr = "OPEN";
  currentState = STATE_OPENING_SCAN;
}

void closeGarage() {
  Serial.println("[ACTION] Closing Garage Servo to 0 degrees...");
  garageServo.write(0);
  garageStatusStr = "CLOSED";
  systemModeStr = "STANDBY";
  currentState = STATE_CLOSED;
  
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, HIGH); 
}

void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) return;

  if (doc.containsKey("command")) {
    const char* cmd = doc["command"];
    if (cmd != NULL) {
      if (strcmp(cmd, "OPEN") == 0) {
        openGarage();
        lastCommandTime = millis();
      } else if (strcmp(cmd, "CLOSE") == 0) {
        closeGarage();
      } else if (strcmp(cmd, "POWER_ON") == 0) {
        systemModeStr = "PARKING_IN";
      } else if (strcmp(cmd, "POWER_OFF") == 0) {
        client.publish(TOPIC_STATUS, "{\"status\":\"OFFLINE\"}", true);
        systemModeStr = "STANDBY";
      }
      shouldPublishTelemetry = true;
    }
  }
}

void setup_wifi() {
  delay(10);
  const char* ssid = "15";
  const char* password = "jqeq6496";
  Serial.print("\nConnecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected. IP: " + WiFi.localIP().toString());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting MQTT...");
    yield(); // Beri waktu OS ESP8266
    String clientId = "Tukir-HW-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass, TOPIC_STATUS, 0, true, "{\"status\":\"OFFLINE\"}")) {
      Serial.println("Connected!");
      client.publish(TOPIC_STATUS, "{\"status\":\"ONLINE\"}", true);
      client.subscribe(TOPIC_CONTROL);
    } else {
      Serial.print("Failed rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5s");
      delay(5000);
    }
  }
}

void bootTestLED() {
  Serial.println("[BOOT] Testing LEDs...");
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_RED, LOW); // Active LOW
  playI2STone(1000, 200);
  delay(800);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, HIGH); // OFF
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_REAR, INPUT);
  pinMode(ECHO_SIDE, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, HIGH); 

  garageServo.attach(SERVO_PIN);
  garageServo.write(0); 

  // i2s_begin(); // Dinonaktifkan untuk mencegah konflik PWM Servo
  // i2s_set_rate(22050); // Dinonaktifkan
  bootTestLED();

  setup_wifi();
  espClient.setInsecure(); 
  espClient.setBufferSizes(512, 512); // Batasi buffer TLS agar RAM tidak habis
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(512); 
  client.setCallback(callback);
}

void publishTelemetry() {
  StaticJsonDocument<512> doc;
  doc["garage_status"]     = garageStatusStr;
  doc["system_mode"]       = systemModeStr;
  doc["wifi_rssi"]         = WiFi.RSSI();
  doc["rear_dist_cm"]      = (int)distRear;
  doc["side_dist_cm"]      = (int)distSide;
  doc["fuzzy_risk_score"]  = fuzzyRiskScore;
  doc["fuzzy_status"]      = fuzzyStatusStr;

  char buffer[512];
  serializeJson(doc, buffer);
  client.publish(TOPIC_TELEMETRY, buffer);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();

  if (shouldPublishTelemetry) {
    publishTelemetry();
    shouldPublishTelemetry = false;
  }

  // Selalu baca sensor untuk update telemetry real-time
  distRear = readUltrasonic(ECHO_REAR);
  delay(10);
  distSide = readUltrasonic(ECHO_SIDE);
  computeFuzzyLogic(distRear, distSide);

  switch (currentState) {
    case STATE_CLOSED:
      systemModeStr = "STANDBY";
      exitValidCounter = 0;
      break;

    case STATE_OPENING_SCAN:
      if (now - lastCommandTime > 1000) {
        distRear = readUltrasonic(ECHO_REAR);
        if (distRear < 12.0) { 
          currentState = STATE_EXITING;
          systemModeStr = "EXITING";
          playI2STone(1200, 300); 
        } else { 
          currentState = STATE_PARKING_IN;
          systemModeStr = "PARKING_IN";
          playI2STone(800, 150);
        }
      }
      break;

    case STATE_PARKING_IN:
      distRear = readUltrasonic(ECHO_REAR);
      delay(15);
      distSide = readUltrasonic(ECHO_SIDE);

      computeFuzzyLogic(distRear, distSide);

      if (fuzzyRiskScore >= 65.0) { 
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_RED, LOW); 
        if (now - lastAudioTime > 120) {
          playI2STone(2000, 50);
          lastAudioTime = now;
        }
      } else if (fuzzyRiskScore >= 30.0) { 
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, HIGH);
        digitalWrite(LED_RED, HIGH); 
        if (now - lastAudioTime > 350) {
          playI2STone(800, 80);
          lastAudioTime = now;
        }
      } else { 
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_RED, HIGH); 
      }

      if (distRear < 4.0) {
        if (!isAutoClosingTimerActive) {
          autoCloseStartTime = now;
          isAutoClosingTimerActive = true;
        } else if (now - autoCloseStartTime >= 5000) {
          closeGarage();
          isAutoClosingTimerActive = false;
        }
      } else {
        isAutoClosingTimerActive = false;
      }
      break;

    case STATE_EXITING:
      distRear = readUltrasonic(ECHO_REAR);
      if (distRear >= 15.0 && distRear < 500.0) {
        exitValidCounter++;
        if (exitValidCounter >= 5) {
          delay(1000);
          closeGarage();
          exitValidCounter = 0;
        }
      } else {
        exitValidCounter = 0;
      }
      break;

    case STATE_CLOSING:
      closeGarage();
      break;
  }

  // Publish Telemetry: Real-time (1.5s) saat aktif, Heartbeat (30s) saat STANDBY/OFF
  unsigned long telemetryInterval = (systemModeStr == "STANDBY") ? 30000 : 1500;
  if (now - lastTelemetryTime > telemetryInterval) {
    lastTelemetryTime = now;
    publishTelemetry();
  }
}
