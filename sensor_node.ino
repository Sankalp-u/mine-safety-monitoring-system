/*
  ============================================================
   Mine Safety Monitoring System - SENSOR NODE (ESP32)
  ============================================================
   Sensors   : DHT11 (temp/humidity), MQ-9 (CO + methane),
               VL53L0X (roof/wall distance -> displacement)
   Indicator : XL-B524SURSYGW (5 mm red + yellow-green, COMMON ANODE)
   Cloud     : ThingSpeak, one channel per node

   Flash the SAME sketch on both nodes - only change NODE_ID.

   ThingSpeak fields (create these in each node's channel):
     field1 = Temperature (C)       field5 = Distance (mm)
     field2 = Humidity (%)          field6 = Displacement from baseline (mm)
     field3 = CO (ppm)              field7 = Status (0 ok, 1 warn, 2 danger)
     field4 = CH4 (ppm)             field8 = Sensor fault (0/1)

   Libraries (Library Manager):
     - "DHT sensor library" by Adafruit (+ "Adafruit Unified Sensor")
     - "Adafruit_VL53L0X" by Adafruit
   Board: ESP32 Dev Module (esp32 by Espressif)
  ============================================================
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_VL53L0X.h>

// ------------------------- USER CONFIG -------------------------
#define NODE_ID 1   // 1 or 2

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

#if NODE_ID == 1
const char* TS_WRITE_KEY = "NODE1_WRITE_API_KEY";
#else
const char* TS_WRITE_KEY = "NODE2_WRITE_API_KEY";
#endif

// ----------------------------- PINS ----------------------------
#define PIN_DHT    4
#define PIN_MQ9    34   // must be an ADC1 pin (ADC2 stops working when WiFi is on)
#define PIN_LED_R  25   // red cathode    (through 120 ohm)
#define PIN_LED_G  26   // green cathode  (through 120 ohm)
#define PIN_SDA    21   // VL53L0X
#define PIN_SCL    22

#define LED_COMMON_ANODE 1   // XL-B524SURSYGW is common anode -> pin LOW = LED ON

// ------------------ THRESHOLDS (same as controller) -------------
const float TEMP_WARN = 35,   TEMP_DANGER = 40;     // deg C
const float HUM_WARN  = 80,   HUM_DANGER  = 90;     // %RH
const float CO_WARN   = 25,   CO_DANGER   = 50;     // ppm
const float CH4_WARN  = 5000, CH4_DANGER  = 10000;  // ppm (0.5 % / 1 % by volume)
const int   DEV_WARN  = 20,   DEV_DANGER  = 50;     // mm change from baseline

// ----------------------------- MQ-9 -----------------------------
const float MQ_VC           = 5.0;                     // module supply
const float MQ_RL           = 10.0;                    // kOhm, load resistor on module (usually "103")
const float DIVIDER_RATIO   = (22.0 + 33.0) / 33.0;    // AO -> 22k -> GPIO34 -> 33k -> GND
const float CLEAN_AIR_RATIO = 9.6;                     // Rs/R0 in clean air (MQ-9 datasheet)
// Curve fits from the MQ-9 sensitivity chart: ppm = a * (Rs/R0)^b
const float CO_A  = 599.65, CO_B  = -2.244;
const float CH4_A = 4269.6, CH4_B = -2.648;

// First run: keep 1, read R0 from Serial Monitor in CLEAN air, paste it below, then set 0.
// (Never auto-calibrate inside a mine - gas present at boot would give a wrong R0.)
#define CALIBRATE_ON_BOOT 1
float R0 = 10.0;   // kOhm

const unsigned long WARMUP_MS = 60000;   // heater warm-up at every boot (new sensor: burn in 24 h+)

// ---------------------------- TIMING ----------------------------
const unsigned long SAMPLE_MS = 2000;    // DHT11 needs >= 1 s between reads
const unsigned long UPLOAD_MS = 20000;   // ThingSpeak free tier: >= 15 s per channel

// ----------------------------------------------------------------
enum { ST_OK = 0, ST_WARN = 1, ST_DANGER = 2 };

DHT dht(PIN_DHT, DHT11);
Adafruit_VL53L0X lox;
bool loxOK = false;
int  baselineMM = -1;

// Latest values + worst value seen since last successful upload
// (so a short gas spike between uploads is never missed)
float tNow = NAN, hNow = NAN;
float coMax = 0, ch4Max = 0;
int   distNow = -1, devMax = 0;
int   statusWorst = ST_OK;
bool  faultAny = false;

int  statusNow = ST_OK;
bool faultNow  = false;

unsigned long lastSample = 0, lastUpload = 0, lastWifiTry = 0;

// ============================ LED ===============================
void ledWrite(bool red, bool green) {
#if LED_COMMON_ANODE
  digitalWrite(PIN_LED_R, red   ? LOW : HIGH);
  digitalWrite(PIN_LED_G, green ? LOW : HIGH);
#else
  digitalWrite(PIN_LED_R, red   ? HIGH : LOW);
  digitalWrite(PIN_LED_G, green ? HIGH : LOW);
#endif
}

// green = OK, amber (red+green) = warning, blinking red = danger,
// blinking amber = sensor fault / warming up
void showStatus(int st, bool fault) {
  bool blink = (millis() / 300) % 2;
  if (st == ST_DANGER)      ledWrite(blink, false);
  else if (st == ST_WARN)   ledWrite(true, true);
  else if (fault)           ledWrite(blink, blink);
  else                      ledWrite(false, true);
}

// ============================ MQ-9 ==============================
float readRs() {
  uint32_t mv = 0;
  for (int i = 0; i < 20; i++) { mv += analogReadMilliVolts(PIN_MQ9); delay(2); }
  float vOut = (mv / 20.0f / 1000.0f) * DIVIDER_RATIO;   // voltage at module AO
  if (vOut < 0.05f) return -1;                           // wire loose / heater off
  if (vOut > MQ_VC - 0.01f) vOut = MQ_VC - 0.01f;
  return MQ_RL * (MQ_VC - vOut) / vOut;                  // kOhm
}

float gasPPM(float ratio, float a, float b) { return a * pow(ratio, b); }

void warmUpAndCalibrate() {
  Serial.printf("MQ-9 warming up for %lu s...\n", WARMUP_MS / 1000);
  unsigned long t0 = millis();
  while (millis() - t0 < WARMUP_MS) { showStatus(ST_OK, true); delay(20); }

#if CALIBRATE_ON_BOOT
  float sum = 0; int n = 0;
  for (int i = 0; i < 50; i++) {
    float rs = readRs();
    if (rs > 0) { sum += rs; n++; }
    showStatus(ST_OK, true);
    delay(100);
  }
  if (n > 0) R0 = (sum / n) / CLEAN_AIR_RATIO;
  Serial.printf(">>> MQ-9 R0 = %.2f kOhm  (paste into sketch, set CALIBRATE_ON_BOOT 0)\n", R0);
#endif
}

// ========================== VL53L0X =============================
int readDistance() {
  if (!loxOK) return -1;
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  if (m.RangeStatus == 4) return -1;    // out of range
  return m.RangeMilliMeter;
}

void captureBaseline() {
  long sum = 0; int n = 0;
  for (int i = 0; i < 30; i++) {
    int d = readDistance();
    if (d > 0) { sum += d; n++; }
    delay(50);
  }
  baselineMM = (n > 0) ? sum / n : -1;
  Serial.printf("VL53L0X baseline = %d mm\n", baselineMM);
}

// ========================= SAMPLING =============================
int levelOf(float v, float warn, float danger) {
  if (isnan(v)) return ST_OK;
  if (v >= danger) return ST_DANGER;
  if (v >= warn)   return ST_WARN;
  return ST_OK;
}

void sampleSensors() {
  bool fault = false;

  tNow = dht.readTemperature();
  hNow = dht.readHumidity();
  if (isnan(tNow) || isnan(hNow)) fault = true;

  float co = 0, ch4 = 0;
  float rs = readRs();
  if (rs > 0 && R0 > 0) {
    float ratio = rs / R0;
    co  = gasPPM(ratio, CO_A, CO_B);
    ch4 = gasPPM(ratio, CH4_A, CH4_B);
  } else {
    fault = true;
  }

  distNow = readDistance();
  int dev = 0;
  if (distNow > 0 && baselineMM > 0) dev = abs(distNow - baselineMM);
  else fault = true;

  int st = ST_OK;
  st = max(st, levelOf(tNow, TEMP_WARN, TEMP_DANGER));
  st = max(st, levelOf(hNow, HUM_WARN, HUM_DANGER));
  st = max(st, levelOf(co, CO_WARN, CO_DANGER));
  st = max(st, levelOf(ch4, CH4_WARN, CH4_DANGER));
  st = max(st, levelOf((float)dev, DEV_WARN, DEV_DANGER));

  statusNow = st;
  faultNow  = fault;

  // worst-case values for the upload window
  coMax       = max(coMax, co);
  ch4Max      = max(ch4Max, ch4);
  devMax      = max(devMax, dev);
  statusWorst = max(statusWorst, st);
  faultAny    = faultAny || fault;

  Serial.printf("[N%d] T=%.1fC H=%.0f%% CO=%.1fppm CH4=%.0fppm D=%dmm dev=%dmm st=%d fault=%d\n",
                NODE_ID, tNow, hNow, co, ch4, distNow, dev, st, fault);
}

// ========================= THINGSPEAK ===========================
void uploadThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) { Serial.println("WiFi down - upload skipped"); return; }

  String url = "http://api.thingspeak.com/update?api_key=" + String(TS_WRITE_KEY);
  if (!isnan(tNow)) url += "&field1=" + String(tNow, 1);
  if (!isnan(hNow)) url += "&field2=" + String(hNow, 1);
  url += "&field3=" + String(coMax, 1);
  url += "&field4=" + String(ch4Max, 0);
  url += "&field5=" + String(distNow);
  url += "&field6=" + String(devMax);
  url += "&field7=" + String(statusWorst);
  url += "&field8=" + String(faultAny ? 1 : 0);

  HTTPClient http;
  http.setTimeout(5000);
  http.begin(url);
  int code = http.GET();
  String body = http.getString();
  http.end();

  if (code == 200 && body != "0") {
    Serial.printf("Uploaded, entry %s\n", body.c_str());
    // reset the window only after a successful upload
    coMax = ch4Max = 0; devMax = 0; statusWorst = ST_OK; faultAny = false;
  } else {
    Serial.printf("Upload failed (HTTP %d, body '%s') - will retry\n", code, body.c_str());
  }
}

// =========================== WIFI ===============================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(300); Serial.print("."); }
  Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " failed (will retry)");
}

void maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    WiFi.reconnect();
  }
}

// =========================== SETUP ==============================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== Mine safety node %d ===\n", NODE_ID);

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  ledWrite(false, false);

  dht.begin();

  Wire.begin(PIN_SDA, PIN_SCL);
  loxOK = lox.begin();
  Serial.println(loxOK ? "VL53L0X OK" : "VL53L0X NOT FOUND - check wiring");

  connectWiFi();
  warmUpAndCalibrate();
  captureBaseline();
}

// ============================ LOOP ==============================
void loop() {
  maintainWiFi();

  if (millis() - lastSample >= SAMPLE_MS) {
    lastSample = millis();
    sampleSensors();
  }

  if (millis() - lastUpload >= UPLOAD_MS) {
    lastUpload = millis();
    uploadThingSpeak();
  }

  showStatus(statusNow, faultNow);   // local alarm works even without WiFi
  delay(10);
}
