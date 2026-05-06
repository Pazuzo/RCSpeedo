#include <WiFi.h>
#include <WebSocketsServer.h>
#include <TinyGPSPlus.h>

// ================= WiFi Access Point =================
const char* ssid = "RCSpeedo";
const char* password = "password";

// ================= WebSocket =================
WebSocketsServer webSocket(81);

// ================= GPS =================
TinyGPSPlus gps;

// ================= DATA STRUCT (17 bytes) =================
struct __attribute__((packed)) Telemetry {
  uint16_t speed;      // 2 bytes (0-655.35 km/h)
  uint16_t maxSpeed;   // 2 bytes
  uint16_t t60;        // 2 bytes (0-655.35 sec)
  uint16_t t100;       // 2 bytes
  uint16_t distance;   // 2 bytes (0-6553.5 m)
  uint8_t sats;        // 1 byte
  uint8_t dummy[6];    // 6 bytes padding
};

Telemetry data;

// ================= Kalman Filter =================
class KalmanFilter {
  public:
    float Q = 0.5;
    float R = 2.0;
    float X = 0;
    float P = 1;

    float update(float m, int sats) {
      if(sats >= 8) R = 0.8;
      else if(sats >= 5) R = 1.5;
      else R = 3.0;

      P += Q;
      float K = P / (P + R);
      X += K * (m - X);
      P *= (1 - K);
      return X;
    }

    void reset() {
      X = 0;
      P = 1;
    }
};

KalmanFilter KF;

// ================= Median Filter =================
float median3(float a, float b, float c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

// ================= Vars =================
float maxSpeed = 0;
float lastRaw = 0;
float lastFiltered = 0;
float lastSpeedForDist = 0;
float rawBuf[3] = {0, 0, 0};
int bufIndex = 0;

unsigned long lastSend = 0;
unsigned long lastUpdate = 0;
unsigned long lastGPSTime = 0;

const float minSpeed = 1.0;
const int minSats = 3;

// ===== Race =====
bool raceStarted = false;
int stableCount = 0;
unsigned long startTime = 0;
float time60 = 0, time100 = 0;
bool done60 = false, done100 = false;
float distance = 0;

// ================= Reset =================
void resetData() {
  maxSpeed = 0;
  time60 = 0;
  time100 = 0;
  distance = 0;
  raceStarted = false;
  stableCount = 0;
  KF.reset();
  Serial.println(">>> RESET <<<");
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n======================================");
  Serial.println("RC Speedometer - ESP32S3+");
  Serial.println("======================================");
  
  // GPS на Serial1 (RX=44, TX=43)
  Serial1.begin(115200, SERIAL_8N1, 44, 43);
  Serial.println("GPS Serial started on pins 44(RX)/43(TX)");
  
  // WiFi Access Point
  WiFi.softAP(ssid, password);
  Serial.print("WiFi AP SSID: ");
  Serial.println(ssid);
  Serial.print("WiFi AP IP: ");
  Serial.println(WiFi.softAPIP());
  
  // WebSocket сървър
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_BIN && length == 1 && payload[0] == 0x01) {
      resetData();
    }
  });
  Serial.println("WebSocket server started on port 81");
  
  lastUpdate = millis();
  Serial.println("======================================");
  Serial.println("READY! Connect to WiFi: RCSpeedo");
  Serial.println("Password: Hermes69$");
  Serial.println("Then open Android app");
  Serial.println("======================================\n");
}

// ================= Loop =================
void loop() {
  // Четене на GPS данни
  while (Serial1.available()) {
    char c = Serial1.read();
    gps.encode(c);
    lastGPSTime = millis();
  }
  
  // WebSocket loop
  webSocket.loop();
  
  unsigned long now = millis();
  
  // Статус на всеки 3 секунди
  static unsigned long lastDebug = 0;
  if (now - lastDebug > 3000) {
    Serial.print("GPS Speed: ");
    Serial.print(gps.speed.kmph());
    Serial.print(" km/h | Sats: ");
    Serial.print(gps.satellites.value());
    Serial.print(" | Clients: ");
    Serial.println(webSocket.connectedClients());
    lastDebug = now;
  }
  
  // Изпращане на данни на всеки 100ms
  if (now - lastSend > 100) {
    
    // Вземане на raw speed
    float raw = gps.speed.isValid() ? gps.speed.kmph() : 0;
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    
    // Проверка за валидност
    if (sats < minSats || (now - lastGPSTime > 2000)) {
      raw = 0;
      KF.reset();
    }
    if (raw < minSpeed) raw = 0;
    
    // Медианен филтър
    rawBuf[bufIndex] = raw;
    bufIndex = (bufIndex + 1) % 3;
    float median = median3(rawBuf[0], rawBuf[1], rawBuf[2]);
    
    // Ограничаване на скоковете
    float diff = abs(median - lastRaw);
    if (diff > 40 && lastRaw > 5) {
      median = lastRaw + (median > lastRaw ? 20 : -20);
    }
    lastRaw = median;
    
    // Kalman филтър
    float speed = KF.update(median, sats);
    
    // IIR филтър за изглаждане
    float alpha = (speed < 30) ? 0.2 : 0.5;
    speed = lastFiltered + alpha * (speed - lastFiltered);
    lastFiltered = speed;
    
    // Максимална скорост
    if (speed > maxSpeed) maxSpeed = speed;
    
    // Изчисляване на дистанция
    float dt = (now - lastUpdate) / 1000.0;
    if (dt <= 0 || dt > 0.5) dt = 0.1;
    lastUpdate = now;
    
    float avgSpeed = (lastSpeedForDist + speed) / 2.0;
    distance += (avgSpeed / 3.6) * dt;
    lastSpeedForDist = speed;
    
    // RACE логика (0-60, 0-100)
    if (speed > 2) {
      stableCount++;
      if (stableCount > 3 && !raceStarted) {
        raceStarted = true;
        startTime = now;
        distance = 0;
        done60 = done100 = false;
        Serial.println(">>> RACE STARTED <<<");
      }
    } else {
      stableCount = 0;
    }
    
    if (raceStarted) {
      if (speed >= 60 && !done60) {
        time60 = (now - startTime) / 1000.0;
        done60 = true;
        Serial.print("0-60: ");
        Serial.println(time60);
      }
      if (speed >= 100 && !done100) {
        time100 = (now - startTime) / 1000.0;
        done100 = true;
        Serial.print("0-100: ");
        Serial.println(time100);
      }
      if (speed < 1 && lastSpeedForDist < 1) {
        raceStarted = false;
        Serial.println(">>> RACE STOPPED <<<");
      }
    }
    
    // Пълнене на структурата
    data.speed = (uint16_t)(speed * 100);
    data.maxSpeed = (uint16_t)(maxSpeed * 100);
    data.t60 = (uint16_t)(time60 * 100);
    data.t100 = (uint16_t)(time100 * 100);
    data.distance = (uint16_t)(distance * 10);
    data.sats = (uint8_t)sats;
    memset(data.dummy, 0, 6);
    
    // Изпращане през WebSocket
    if (webSocket.connectedClients() > 0) {
      webSocket.broadcastBIN((uint8_t*)&data, 17);
    }
    
    lastSend = now;
  }
}