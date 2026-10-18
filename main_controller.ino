/*
  ============================================================
   Mine Safety Monitoring System - MAIN CONTROLLER (ESP32)
  ============================================================
   Reads both node channels from ThingSpeak every 20 s, checks
   thresholds and node freshness, and raises alarms.

   Outputs : SSD1306 0.96" OLED (I2C, 4-pin), GY-GPS6MV2 (NEO-6M),
             buzzer, XL-B524SURSYGW red/green status LED

   LED / buzzer:
     green, silent           -> all normal
     amber, beep every 2 s   -> warning / node offline / sensor fault
     blinking red, fast beep -> DANGER

   Libraries (Library Manager):
     - "Adafruit SSD1306" + "Adafruit GFX Library"
     - "TinyGPSPlus" by Mikal Hart
     - "ArduinoJson" by Benoit Blanchon (v7)
   Board: ESP32 Dev Module
  ============================================================
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <time.h>

// ------------------------- USER CONFIG -------------------------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

const unsigned long CH_ID[2]       = { 1111111, 2222222 };                    // channel IDs
const char*         CH_READ_KEY[2] = { "NODE1_READ_API_KEY", "NODE2_READ_API_KEY" };

// ----------------------------- PINS ----------------------------
#define PIN_SDA     21
#define PIN_SCL     22
#define PIN_GPS_RX  16   // ESP32 RX2  <- GPS TX
#define PIN_GPS_TX  17   // ESP32 TX2  -> GPS RX
#define PIN_BUZZER  13
#define PIN_LED_R   25
#define PIN_LED_G   26

#define LED_COMMON_ANODE    1   // XL-B524SURSYGW: LOW = ON
#define BUZZER_ACTIVE_HIGH  1   // set 0 if your 3-pin buzzer module is active-LOW
#define OLED_ADDR           0x3C

// ------------------ THRESHOLDS (same as nodes) ------------------
const float TEMP_WARN = 35,   TEMP_DANGER = 40;
const float HUM_WARN  = 80,   HUM_DANGER  = 90;
const float CO_WARN   = 25,   CO_DANGER   = 50;
const float CH4_WARN  = 5000, CH4_DANGER  = 10000;
const float DEV_WARN  = 20,   DEV_DANGER  = 50;

// ---------------------------- TIMING ----------------------------
const unsigned long POLL_MS     = 20000;  // read ThingSpeak every 20 s
const long          MAX_AGE_S   = 90;     // data older than this = node offline
const int           STALE_POLLS = 4;      // fallback if NTP time not available
const unsigned long PAGE_MS     = 3000;   // OLED page rotation

// ----------------------------------------------------------------
enum { ST_OK = 0, ST_WARN = 1, ST_DANGER = 2 };
const char* LEVEL_NAME[] = { "NORMAL", "WARNING", "DANGER" };

struct Node {
  float t = NAN, h = NAN, co = NAN, ch4 = NAN, dev = NAN;
  int   dist = -1, nodeStatus = 0, fault = 0;
  long  lastEntry = -1;
  int   staleCount = 0, failCount = 0;
  bool  online = false;
  int   level = ST_WARN;
  String reason = "Waiting data";
};

Node nodes[2];
int  overall = ST_WARN, prevOverall = ST_OK;

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOK = false;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

unsigned long lastPoll = 0, lastDraw = 0, lastWifiTry = 0;
bool firstPoll = true;

// ======================== LED / BUZZER ==========================
void ledWrite(bool red, bool green) {
#if LED_COMMON_ANODE
  digitalWrite(PIN_LED_R, red   ? LOW : HIGH);
  digitalWrite(PIN_LED_G, green ? LOW : HIGH);
#else
  digitalWrite(PIN_LED_R, red   ? HIGH : LOW);
  digitalWrite(PIN_LED_G, green ? HIGH : LOW);
#endif
}

void buzz(bool on) {
#if BUZZER_ACTIVE_HIGH
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
#else
  digitalWrite(PIN_BUZZER, on ? LOW : HIGH);
#endif
}

void updateOutputs() {
  unsigned long ms = millis();
  switch (overall) {
    case ST_DANGER:
      ledWrite((ms / 250) % 2, false);
      buzz((ms / 200) % 2);
      break;
    case ST_WARN:
      ledWrite(true, true);
      buzz((ms % 2000) < 120);
      break;
    default:
      ledWrite(false, true);
      buzz(false);
  }
}

// ========================= THINGSPEAK ===========================
float fieldF(JsonDocument& doc, const char* key) {
  const char* s = doc[key];
  return (s && *s) ? atof(s) : NAN;
}

time_t parseISO(const char* s) {            // "2026-10-17T10:15:30Z" (UTC)
  struct tm tm = {};
  if (sscanf(s, "%d-%d-%dT%d:%d:%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
             &tm.tm_hour, &tm.tm_min, &tm.tm_sec) != 6) return 0;
  tm.tm_year -= 1900;
  tm.tm_mon  -= 1;
  return mktime(&tm);                       // TZ is UTC (configTime(0,0,...))
}

bool fetchNode(int i) {
  if (WiFi.status() != WL_CONNECTED) return false;

  String url = "http://api.thingspeak.com/channels/" + String(CH_ID[i]) +
               "/feeds/last.json?api_key=" + String(CH_READ_KEY[i]);
  HTTPClient http;
  http.setTimeout(5000);
  if (!http.begin(url)) return false;
  int code = http.GET();
  if (code != 200) { http.end(); return false; }
  String payload = http.getString();
  http.end();

  JsonDocument doc;
  if (deserializeJson(doc, payload) || !doc.is<JsonObject>()) return false;  // "-1" = empty channel

  Node& n = nodes[i];
  n.t          = fieldF(doc, "field1");
  n.h          = fieldF(doc, "field2");
  n.co         = fieldF(doc, "field3");
  n.ch4        = fieldF(doc, "field4");
  float d      = fieldF(doc, "field5");
  n.dist       = isnan(d) ? -1 : (int)d;
  n.dev        = fieldF(doc, "field6");
  float s      = fieldF(doc, "field7");
  n.nodeStatus = isnan(s) ? 0 : (int)s;
  float f      = fieldF(doc, "field8");
  n.fault      = isnan(f) ? 0 : (int)f;

  // ---- freshness: is the node still uploading? ----
  long entry = doc["entry_id"] | -1L;
  if (entry != n.lastEntry) { n.staleCount = 0; n.lastEntry = entry; }
  else n.staleCount++;

  time_t now = time(nullptr);
  const char* created = doc["created_at"];
  if (now > 1700000000 && created) {
    n.online = (now - parseISO(created)) <= MAX_AGE_S;
  } else {
    n.online = n.staleCount < STALE_POLLS;
  }
  return true;
}

// ========================= ANOMALY CHECK ========================
void check(Node& n, float v, float warn, float danger, const char* label, const char* unit) {
  if (isnan(v)) return;
  int l = (v >= danger) ? ST_DANGER : (v >= warn) ? ST_WARN : ST_OK;
  if (l > n.level) {
    n.level  = l;
    n.reason = String(label) + " " + String(v, 0) + unit;
  }
}

void evaluate(Node& n) {
  n.level  = ST_OK;
  n.reason = "All normal";
  if (!n.online) { n.level = ST_WARN; n.reason = "Node offline"; return; }

  check(n, n.co,  CO_WARN,   CO_DANGER,   "CO",   "ppm");
  check(n, n.ch4, CH4_WARN,  CH4_DANGER,  "CH4",  "ppm");
  check(n, n.dev, DEV_WARN,  DEV_DANGER,  "Roof", "mm");
  check(n, n.t,   TEMP_WARN, TEMP_DANGER, "Temp", "C");
  check(n, n.h,   HUM_WARN,  HUM_DANGER,  "Hum",  "%");

  if (n.nodeStatus > n.level) { n.level = n.nodeStatus; n.reason = "Node local alarm"; }
  if (n.level == ST_OK && n.fault) { n.level = ST_WARN; n.reason = "Sensor fault"; }
}

void pollNodes() {
  for (int i = 0; i < 2; i++) {
    if (fetchNode(i)) nodes[i].failCount = 0;
    else if (++nodes[i].failCount >= STALE_POLLS) nodes[i].online = false;
    evaluate(nodes[i]);
  }

  prevOverall = overall;
  overall = max(nodes[0].level, nodes[1].level);

  // log every escalation with GPS position of the control unit
  if (overall > prevOverall) {
    Serial.printf("\n!!! %s !!!\n", LEVEL_NAME[overall]);
    for (int i = 0; i < 2; i++)
      Serial.printf("  Node %d: %s - %s\n", i + 1, LEVEL_NAME[nodes[i].level], nodes[i].reason.c_str());
    if (gps.location.isValid())
      Serial.printf("  Control unit @ %.6f, %.6f\n", gps.location.lat(), gps.location.lng());
  }

  for (int i = 0; i < 2; i++) {
    Node& n = nodes[i];
    Serial.printf("[N%d] %s T=%.1f H=%.0f CO=%.1f CH4=%.0f D=%d dev=%.0f -> %s (%s)\n",
                  i + 1, n.online ? "on " : "off", n.t, n.h, n.co, n.ch4, n.dist, n.dev,
                  LEVEL_NAME[n.level], n.reason.c_str());
  }
}

// ============================ OLED ==============================
void printVal(const char* label, float v, int dec, const char* unit) {
  oled.print(label);
  if (isnan(v)) oled.println("--");
  else { oled.print(v, dec); oled.println(unit); }
}

void drawNodePage(int i) {
  Node& n = nodes[i];
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.printf("NODE %d   %s\n", i + 1, n.online ? "ONLINE" : "OFFLINE");
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
  oled.setCursor(0, 12);
  printVal("Temp : ", n.t, 1, " C");
  printVal("Hum  : ", n.h, 0, " %");
  printVal("CO   : ", n.co, 0, " ppm");
  printVal("CH4  : ", n.ch4, 0, " ppm");
  printVal("Roof : ", n.dev, 0, " mm shift");
  oled.printf("%s: %s", LEVEL_NAME[n.level], n.reason.c_str());
}

void drawGpsPage() {
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("CONTROL UNIT");
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
  oled.setCursor(0, 12);
  if (gps.location.isValid()) {
    oled.printf("Lat: %.6f\n", gps.location.lat());
    oled.printf("Lng: %.6f\n", gps.location.lng());
  } else {
    oled.println("GPS: searching...");
    oled.println("");
  }
  oled.printf("Sats: %d\n", gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  if (gps.time.isValid())
    oled.printf("UTC : %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
  else
    oled.println("UTC : --");
  oled.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "connected" : "DOWN");
  oled.printf("System: %s", LEVEL_NAME[overall]);
}

void drawAlertPage() {
  bool flash = (millis() / 500) % 2;
  if (flash) oled.fillRect(0, 0, 128, 20, SSD1306_WHITE);
  oled.setTextColor(flash ? SSD1306_BLACK : SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(28, 3);
  oled.print("DANGER");
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 26);
  for (int i = 0; i < 2; i++) {
    if (nodes[i].level == ST_DANGER)
      oled.printf("N%d: %s\n", i + 1, nodes[i].reason.c_str());
  }
  oled.println("EVACUATE AREA");
}

void drawScreen() {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  if (overall == ST_DANGER) {
    drawAlertPage();
  } else {
    int page = (millis() / PAGE_MS) % 3;
    if (page < 2) drawNodePage(page);
    else          drawGpsPage();
  }
  oled.display();
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
  Serial.println("\n=== Mine safety main controller ===");

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  ledWrite(false, false);
  buzz(false);

  Wire.begin(PIN_SDA, PIN_SCL);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOK) Serial.println("OLED not found - check wiring / address (0x3C or 0x3D)");
  else {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 20);
    oled.println("Mine Safety System");
    oled.println("Connecting...");
    oled.display();
  }

  gpsSerial.setRxBufferSize(1024);   // HTTP calls block; bigger buffer keeps NMEA intact
  gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.google.com");   // UTC, used for data-age check

  // self test: short beep + LED colours
  ledWrite(true, false); buzz(true);  delay(150);
  ledWrite(false, true); buzz(false); delay(300);
}

// ============================ LOOP ==============================
void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  maintainWiFi();

  if (firstPoll || millis() - lastPoll >= POLL_MS) {
    firstPoll = false;
    lastPoll = millis();
    pollNodes();
  }

  updateOutputs();

  if (millis() - lastDraw >= 200) {
    lastDraw = millis();
    drawScreen();
  }
}
