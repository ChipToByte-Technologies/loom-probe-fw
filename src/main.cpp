// Loom probe v2 — ChipToByte
// Inputs (LA-WMS input-LED cathodes, LED lit = LOW): PICK=GPIO4 (red), WARP?=GPIO5 (green), WEFT=GPIO18 (blue)
// Publishes filtered loom events over MQTT/TLS, buffers them while offline, accepts OTA/reboot commands.
// Topics: loom/<id>/status (retained, LWT) | loom/<id>/evt | loom/<id>/sum (every 10 s) | loom/<id>/cmd (in)
#include <Arduino.h>
#ifndef PROBE_FW_VERSION
#define PROBE_FW_VERSION "dev"
#endif
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPUpdate.h>
#include <time.h>
#include <WiFiMulti.h>
#include <Preferences.h>
#include <WebServer.h>
#include <esp_ota_ops.h>
#include <Update.h>
// secrets.h is OPTIONAL: present on USB builds (seeds the ESP32's flash storage), absent on OTA builds.
#if __has_include("secrets.h") && !defined(NO_SECRETS)
#include "secrets.h"
#define HAVE_SECRETS 1
#endif
// Site finding: each revolution gives a close PAIR of pulses (~20-25 ms apart) then a long gap.
// Counting with a 40 ms minimum gap = exactly 1 count per revolution, immune to the pair and to RF noise.
#ifdef PULSES_PER_REV
#undef PULSES_PER_REV
#endif
#define PULSES_PER_REV 1

// ---------------- rules from site measurements (docs/loom-signals.md) ----------------
static const int PIN_PICK = 4, PIN_WARP = 5, PIN_WEFT = 18;
static const uint32_t STOP_GAP_MS     = 500;   // no pulse for 0.5 s = stopped
static const uint32_t DEBOUNCE_MS     = 200;   // warp/weft must be stable this long
static const uint32_t BLANK_MS        = 3000;  // ignore warp/weft for 3 s after pulses start
static const uint32_t MIN_RUN_PULSES  = 10;    // fewer = inching/jog, not a run
static const uint32_t SUM_PERIOD_MS   = 10000;
static const uint32_t STOP_SETTLE_MS  = 1500;  // warp drops at/after the stop; weft ~1 pick before: decide reason after settling

// ---------------- pulse capture ----------------
static const uint32_t MIN_PULSE_GAP_US = 40000;
volatile uint32_t pulseCount = 0, lastPulseUs = 0, prevPulseUs = 0, periodUs = 0;
volatile uint32_t rawCount = 0, lastRawUs = 0, minRawGapUs = 0xFFFFFFFF;
void IRAM_ATTR onPick() {
  uint32_t t = micros();
  uint32_t rg = t - lastRawUs;
  if (rg >= 3000) { rawCount++; if (rg < minRawGapUs) minRawGapUs = rg; lastRawUs = t; }   // raw (3 ms) for diagnostics
  if (t - lastPulseUs < MIN_PULSE_GAP_US) return;   // reject glitches: real pulses are >= 40 ms apart (<=750 rpm at 2/rev)
  prevPulseUs = lastPulseUs; lastPulseUs = t; periodUs = t - prevPulseUs; pulseCount++;
}

// ---------------- debounced level input ----------------
struct Deb { int pin; bool on; bool raw; uint32_t since; };
static Deb warp = {PIN_WARP, true, true, 0}, weft = {PIN_WEFT, true, true, 0};
static bool readOn(int pin) { return digitalRead(pin) == LOW; }
static bool updateDeb(Deb &d, uint32_t now) {   // returns true if stable state changed
  bool r = readOn(d.pin);
  if (r != d.raw) { d.raw = r; d.since = now; }
  if (d.raw != d.on && now - d.since >= DEBOUNCE_MS) { d.on = d.raw; return true; }
  return false;
}

// ---------------- loom state machine ----------------
enum State { STOPPED, STARTING, RUNNING, STOPPING };
static State st = STOPPED;
static uint32_t runStartMs = 0, runStartPulses = 0, stopStartMs = 0, stopDetectMs = 0, stopPulses = 0;
static const char* stopReason = "unknown";      // reason being detected during the current run
static const char* lastStopReason = "startup";  // reason of the stop that a new run ends
static uint32_t shiftPulses = 0, runMsTotal = 0;


// ---------------- configuration in flash storage (NVS), survives OTA ----------------
Preferences prefs;
struct Cfg { String ssid1, pass1, ssid2, pass2, host, user, pass, id; uint16_t port; } cfg;
static void loadCfg() {
  prefs.begin("cfg", false);
#ifdef HAVE_SECRETS
  // USB build: write compiled secrets into storage (keeps a backup network set over MQTT if secrets has none)
  prefs.putString("ssid1", WIFI_SSID); prefs.putString("pass1", WIFI_PASS);
#ifdef WIFI_SSID2
  prefs.putString("ssid2", WIFI_SSID2); prefs.putString("pass2", WIFI_PASS2);
#endif
  prefs.putString("host", MQTT_HOST); prefs.putUShort("port", MQTT_PORT);
  prefs.putString("user", MQTT_USER); prefs.putString("mpass", MQTT_PASS); prefs.putString("id", DEVICE_ID);
#endif
  cfg.ssid1 = prefs.getString("ssid1", ""); cfg.pass1 = prefs.getString("pass1", "");
  cfg.ssid2 = prefs.getString("ssid2", ""); cfg.pass2 = prefs.getString("pass2", "");
  cfg.host = prefs.getString("host", ""); cfg.port = prefs.getUShort("port", 8883);
  cfg.user = prefs.getString("user", ""); cfg.pass = prefs.getString("mpass", ""); cfg.id = prefs.getString("id", "probe");
}

// ---------------- safe OTA: new firmware must reach MQTT in 5 min, else boot back ----------------
static const uint32_t OTA_TRIAL_MS = 300000;
static bool otaTrial = false;
static void otaBootCheck() {
  otaTrial = prefs.getBool("ota_trial", false);
  if (!otaTrial) return;
  uint8_t boots = prefs.getUChar("ota_boots", 0) + 1; prefs.putUChar("ota_boots", boots);
  Serial.printf("OTA trial boot %u\n", boots);
  if (boots > 3) { Serial.println("OTA trial: crash loop, rolling back"); 
    const esp_partition_t* prev = esp_ota_get_next_update_partition(NULL);   // the other app slot = previous firmware
    prefs.putBool("ota_trial", false); prefs.putUChar("ota_boots", 0);
    if (prev) esp_ota_set_boot_partition(prev); delay(200); ESP.restart(); }
}
static void otaConfirm() {
  if (!otaTrial) return;
  otaTrial = false; prefs.putBool("ota_trial", false); prefs.putUChar("ota_boots", 0);
  Serial.println("OTA trial confirmed (MQTT reached)");
}
static void otaTrialTimeout() {
  if (otaTrial && millis() > OTA_TRIAL_MS) {
    Serial.println("OTA trial: no MQTT in 5 min, rolling back");
    const esp_partition_t* prev = esp_ota_get_next_update_partition(NULL);
    prefs.putBool("ota_trial", false); prefs.putUChar("ota_boots", 0);
    if (prev) esp_ota_set_boot_partition(prev); delay(200); ESP.restart();
  }
}

// ---------------- setup hotspot: no Wi-Fi for 10 min -> AP "LoomProbe-<id>" with a form ----------------
static const uint32_t PORTAL_AFTER_MS = 600000;
WebServer web(80); static bool portalOn = false; static uint32_t wifiDownSince = 0;
static void startPortal() {
  String ap = "LoomProbe-" + cfg.id; WiFi.mode(WIFI_AP_STA); WiFi.softAP(ap.c_str(), "chiptobyte");
  portalOn = true; Serial.printf("Setup hotspot started: %s (pass chiptobyte) http://192.168.4.1 (login admin)\n", ap.c_str());
}

static const char* OTA_LATEST_URL = "https://github.com/ChipToByte-Technologies/loom-probe-fw/releases/latest/download/firmware.bin";

extern PubSubClient mqtt;
// ---------------- local web page (LAN): status + firmware upload at http://<ip>/update ----------------
// Login: user "admin", password = web_pass in storage (default "chiptobyte"; change with MQTT cmd "webpass <new>")
static String webPass() { return prefs.getString("webpass", "chiptobyte"); }
static bool webAuth() { if (!web.authenticate("admin", webPass().c_str())) { web.requestAuthentication(); return false; } return true; }
static void setupWeb() {
  web.on("/", HTTP_GET, []() {
    if (!webAuth()) return;
    const esp_partition_t* run = esp_ota_get_running_partition();
    String h = "<html><head><meta name='viewport' content='width=device-width'></head><body style='font-family:sans-serif;max-width:480px;margin:20px auto'>"
      "<h3>Loom probe " + cfg.id + "</h3><p>Firmware <b>" PROBE_FW_VERSION "</b> (" + String(run ? run->label : "?") + ")<br>"
      "Wi-Fi " + WiFi.SSID() + " " + String(WiFi.RSSI()) + " dBm<br>MQTT " + (mqtt.connected() ? "connected" : "offline") + "</p>"
      "<h4>Upload firmware (.bin)</h4><form method='POST' action='/update' enctype='multipart/form-data'>"
      "<input type='file' name='fw' accept='.bin'><br><br><button>Upload &amp; install</button></form>"
      "<h4>Wi-Fi</h4><form method='POST' action='/save'>SSID<br><input name='s'><br>Password<br><input name='p' type='password'><br><br>"
      "<button>Save &amp; restart</button></form></body></html>";
    web.send(200, "text/html", h); });
  web.on("/save", HTTP_POST, []() {
    if (!webAuth()) return;
    prefs.putString("ssid2", cfg.ssid1); prefs.putString("pass2", cfg.pass1);      // old primary becomes backup
    prefs.putString("ssid1", web.arg("s")); prefs.putString("pass1", web.arg("p"));
    web.send(200, "text/plain", "Saved. Restarting..."); delay(500); ESP.restart(); });
  web.on("/update", HTTP_POST,
    []() {                                   // after upload finished
      if (!webAuth()) return;
      bool ok = !Update.hasError();
      web.send(200, "text/plain", ok ? "Update OK. Restarting..." : "Update FAILED");
      if (ok) { prefs.putBool("ota_trial", true); prefs.putUChar("ota_boots", 0); delay(500); ESP.restart(); }
    },
    []() {                                   // upload chunks
      HTTPUpload& u = web.upload();
      if (!web.authenticate("admin", webPass().c_str())) return;
      if (u.status == UPLOAD_FILE_START) { Serial.printf("Web OTA: %s\n", u.filename.c_str()); Update.begin(UPDATE_SIZE_UNKNOWN); }
      else if (u.status == UPLOAD_FILE_WRITE) Update.write(u.buf, u.currentSize);
      else if (u.status == UPLOAD_FILE_END) { if (Update.end(true)) Serial.printf("Web OTA: %u bytes OK\n", u.totalSize); else Update.printError(Serial); }
    });
  web.begin(); Serial.println("Web page on port 80 (user admin)");
}


// ---------------- status LED on GPIO2 ----------------
// Blink on every MQTT publish; "breathing" fade while Wi-Fi is disconnected; off otherwise.
static const int LED_PIN = 2, LED_CH = 0;
static uint32_t ledBlinkUntil = 0;
static void ledInit() { ledcSetup(LED_CH, 5000, 8); ledcAttachPin(LED_PIN, LED_CH); ledcWrite(LED_CH, 0); }
static void ledPulse() { ledBlinkUntil = millis() + 40; }
static void ledUpdate() {
  uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) {                 // breathe: 2 s up/down, gamma-ish curve
    uint32_t ph = now % 2000, v = ph < 1000 ? ph * 255 / 1000 : (2000 - ph) * 255 / 1000;
    ledcWrite(LED_CH, v * v / 255); return;
  }
  ledcWrite(LED_CH, now < ledBlinkUntil ? 255 : 0);
}

// ---------------- MQTT + offline buffer ----------------
WiFiClientSecure tls; PubSubClient mqtt(tls); WiFiMulti wifiMulti;
static bool pub(const char* t, const char* p, bool retained = false) { bool ok = mqtt.publish(t, p, retained); if (ok) ledPulse(); return ok; }
static char tEvt[64], tSum[64], tStatus[64], tCmd[64];
static const int QN = 120; static String q[QN]; static int qHead = 0, qLen = 0;
static void enqueue(const String &s) { int i = (qHead + qLen) % QN; if (qLen == QN) { qHead = (qHead + 1) % QN; qLen--; } q[i] = s; qLen++; }
static void flushQueue() { while (qLen && mqtt.connected()) { if (!pub(tEvt, q[qHead].c_str())) break; qHead = (qHead + 1) % QN; qLen--; } }

static String isoNow() {
  time_t t = time(nullptr); if (t < 1700000000) return String("ms:") + millis();
  struct tm tmv; localtime_r(&t, &tmv); char b[32]; strftime(b, sizeof b, "%Y-%m-%dT%H:%M:%S+05:30", &tmv); return b;
}
static void evt(const char* type, const String &extra) {
  String s = String("{\"ts\":\"") + isoNow() + "\",\"type\":\"" + type + "\"" + (extra.length() ? "," + extra : "") + "}";
  Serial.println("EVT " + s);
  if (mqtt.connected()) { if (!pub(tEvt, s.c_str())) enqueue(s); } else enqueue(s);
}

static void doOta(const String &url) {
  evt("ota_start", "\"url\":\"" + url + "\""); mqtt.loop();
  prefs.putBool("ota_trial", true); prefs.putUChar("ota_boots", 0);     // new image must prove itself
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return r;
  if (url.startsWith("https://")) { WiFiClientSecure c; c.setInsecure(); r = httpUpdate.update(c, url); }
  else { WiFiClient c; r = httpUpdate.update(c, url); }
  prefs.putBool("ota_trial", false);                                     // only reached on failure
  evt("ota_fail", "\"err\":\"" + String(httpUpdate.getLastErrorString()) + "\"");
  (void)r;
}
static void onCmd(char* topic, byte* p, unsigned int n) {
  String c; for (unsigned i = 0; i < n; i++) c += (char)p[i]; c.trim();
  Serial.println("CMD " + c);
  if (c == "ping") evt("pong", "\"fw\":\"" PROBE_FW_VERSION "\",\"queued\":" + String(qLen));
  else if (c == "reboot") { evt("reboot", ""); mqtt.loop(); delay(300); ESP.restart(); }
  else if (c == "ota latest") doOta(OTA_LATEST_URL);
  else if (c.startsWith("webpass ") && c.length() >= 16) { prefs.putString("webpass", c.substring(8)); evt("cfg", "\"webpass\":\"changed\""); }
  else if (c.startsWith("ota ")) doOta(c.substring(4));
  else if (c.startsWith("wifi2 ")) {          // "wifi2 <ssid>|<password>" sets the BACKUP network (primary untouched = safe)
    String a = c.substring(6); int k = a.indexOf('|');
    if (k > 0) { prefs.putString("ssid2", a.substring(0, k)); prefs.putString("pass2", a.substring(k + 1));
      evt("cfg", "\"wifi2\":\"" + a.substring(0, k) + "\",\"note\":\"restart to apply\""); }
  }
  else if (c == "wifi_swap") {                // backup becomes primary; both are still tried, so no lock-out
    String s1 = cfg.ssid1, p1 = cfg.pass1;
    prefs.putString("ssid1", cfg.ssid2); prefs.putString("pass1", cfg.pass2);
    prefs.putString("ssid2", s1); prefs.putString("pass2", p1);
    evt("cfg", "\"wifi_swap\":true"); mqtt.loop(); delay(300); ESP.restart();
  }
  else if (c == "info") {
    const esp_partition_t* run = esp_ota_get_running_partition();
    evt("info", "\"fw\":\"" PROBE_FW_VERSION "\",\"part\":\"" + String(run ? run->label : "?") + "\",\"ssid\":\"" + WiFi.SSID() +
        "\",\"wifi1\":\"" + cfg.ssid1 + "\",\"wifi2\":\"" + cfg.ssid2 + "\",\"rssi\":" + String(WiFi.RSSI()) + ",\"heap\":" + String(ESP.getFreeHeap()));
  }
}

static uint32_t lastConnTry = 0;
static uint32_t lastWifiLog = 0; static bool wifiWasUp = false;
static void ensureConn() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiWasUp) { wifiWasUp = false; wifiDownSince = millis(); Serial.println("WiFi lost"); }
    static uint32_t lastMulti = 0; if (millis() - lastMulti > 20000) { lastMulti = millis(); wifiMulti.run(1500); }   // short + infrequent so the LED keeps breathing
    if (!portalOn && millis() - wifiDownSince > PORTAL_AFTER_MS) startPortal();
    if (millis() - lastWifiLog > 10000) { lastWifiLog = millis(); Serial.printf("WiFi connecting (%s / %s) status %d\n", cfg.ssid1.c_str(), cfg.ssid2.c_str(), WiFi.status()); }
    return;
  }
  if (!wifiWasUp) { wifiWasUp = true; Serial.printf("WiFi up: ip=%s rssi=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI()); }
  if (mqtt.connected()) return;
  if (millis() - lastConnTry < 5000) return; lastConnTry = millis();
  String cid = String("loom-") + cfg.id + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(cid.c_str(), cfg.user.c_str(), cfg.pass.c_str(), tStatus, 1, true, "{\"online\":false}")) {
    String s = String("{\"online\":true,\"fw\":\"" PROBE_FW_VERSION "\",\"ip\":\"") + WiFi.localIP().toString() + "\",\"rssi\":" + WiFi.RSSI() + "}";
    pub(tStatus, s.c_str(), true);
    mqtt.subscribe(tCmd, 1);
    Serial.println("MQTT connected"); otaConfirm(); flushQueue();
  } else Serial.printf("MQTT connect failed rc=%d\n", mqtt.state());
}

void setup() {
  Serial.begin(115200); delay(200);
  ledInit();
  pinMode(PIN_PICK, INPUT_PULLUP); pinMode(PIN_WARP, INPUT_PULLUP); pinMode(PIN_WEFT, INPUT_PULLUP);
  warp.on = warp.raw = readOn(PIN_WARP); weft.on = weft.raw = readOn(PIN_WEFT);
  attachInterrupt(PIN_PICK, onPick, FALLING);
  loadCfg(); otaBootCheck();
  const char* id = cfg.id.c_str();
  snprintf(tEvt, sizeof tEvt, "loom/%s/evt", id);  snprintf(tSum, sizeof tSum, "loom/%s/sum", id);
  snprintf(tStatus, sizeof tStatus, "loom/%s/status", id); snprintf(tCmd, sizeof tCmd, "loom/%s/cmd", id);
  const esp_partition_t* run = esp_ota_get_running_partition();
  Serial.printf("\n=== LOOM PROBE v%s === id=%s ppr=%d part=%s secrets=%s\n", PROBE_FW_VERSION, id, PULSES_PER_REV, run ? run->label : "?",
#ifdef HAVE_SECRETS
    "compiled"
#else
    "from-storage"
#endif
  );
  WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true);
  if (cfg.ssid1.length()) wifiMulti.addAP(cfg.ssid1.c_str(), cfg.pass1.c_str());
  if (cfg.ssid2.length()) wifiMulti.addAP(cfg.ssid2.c_str(), cfg.pass2.c_str());
  wifiDownSince = millis(); wifiMulti.run(5000);
  setupWeb();
  configTzTime("IST-5:30", "pool.ntp.org", "time.google.com");
  tls.setInsecure();                 // probe only: encrypted, server not verified. Production pins the CA.
  mqtt.setServer(cfg.host.c_str(), cfg.port); mqtt.setBufferSize(512); mqtt.setKeepAlive(30); mqtt.setCallback(onCmd);
  stopStartMs = millis();
  evt("boot", "\"fw\":\"" PROBE_FW_VERSION "\",\"warp_on\":" + String(warp.on) + ",\"weft_on\":" + String(weft.on));
}

static uint32_t lastSum = 0, lastSumPulses = 0, lastSumRaw = 0;
void loop() {
  uint32_t now = millis();
  ensureConn(); mqtt.loop(); flushQueue(); otaTrialTimeout(); web.handleClient(); ledUpdate();

  noInterrupts(); uint32_t pc = pulseCount, lp = lastPulseUs, per = periodUs; interrupts();
  uint32_t sinceLastPulseMs = (micros() - lp) / 1000;
  bool pulsing = pc > 0 && sinceLastPulseMs < STOP_GAP_MS;

  bool warpCh = updateDeb(warp, now), weftCh = updateDeb(weft, now);

  switch (st) {
    case STOPPED:
      if (pulsing) { st = STARTING; runStartMs = now; runStartPulses = pc; }
      break;
    case STARTING:   // wait for MIN_RUN_PULSES; otherwise it was inching
      if (!pulsing) { st = STOPPED; break; }                      // jog — stop continues, no event
      if (pc - runStartPulses >= MIN_RUN_PULSES) {
        st = RUNNING;
        evt("run", "\"stopped_s\":" + String((runStartMs - stopStartMs) / 1000) + ",\"after\":\"" + lastStopReason + "\"");
      }
      break;
    case RUNNING: {
      bool blanked = now - runStartMs < BLANK_MS;
      if (!blanked && weftCh && !weft.on) stopReason = "weft";
      if (!blanked && warpCh && !warp.on) stopReason = "warp";
      if (!pulsing) { st = STOPPING; stopDetectMs = now; stopStartMs = now - sinceLastPulseMs; stopPulses = pc; }
    } break;
    case STOPPING: {
      if (weftCh && !weft.on && strcmp(stopReason, "warp")) stopReason = "weft";
      if (warpCh && !warp.on && strcmp(stopReason, "weft")) stopReason = "warp";
      if (pulsing) { st = RUNNING; break; }                      // brief hiccup, still running
      if (now - stopDetectMs >= STOP_SETTLE_MS) {
        st = STOPPED;
        if (strcmp(stopReason, "weft") && strcmp(stopReason, "warp")) stopReason = (!weft.on) ? "weft" : (!warp.on) ? "warp" : "manual";
        uint32_t runP = stopPulses - runStartPulses; runMsTotal += stopStartMs - runStartMs;
        evt("stop", "\"reason\":\"" + String(stopReason) + "\",\"run_s\":" + String((stopStartMs - runStartMs) / 1000) +
                    ",\"picks\":" + String(runP / PULSES_PER_REV));
        lastStopReason = stopReason; stopReason = "unknown";
      }
    } break;
  }

  if (now - lastSum >= SUM_PERIOD_MS) {
    noInterrupts(); uint32_t rc = rawCount, mg = minRawGapUs; minRawGapUs = 0xFFFFFFFF; interrupts();
    uint32_t dr = rc - lastSumRaw; lastSumRaw = rc;
    uint32_t dp = pc - lastSumPulses; lastSumPulses = pc; lastSum = now;
    float rpm = (st == RUNNING) ? (dp * 60000.0f / (float)SUM_PERIOD_MS) / PULSES_PER_REV : 0;   // average over the window
    String s = String("{\"ts\":\"") + isoNow() + "\",\"state\":\"" + (st == RUNNING || st == STOPPING ? "running" : "stopped") +
      "\",\"rpm\":" + String(rpm, 1) + ",\"picks_10s\":" + String(dp / PULSES_PER_REV) +
      ",\"picks_total\":" + String(pc / PULSES_PER_REV) + ",\"warp_on\":" + String(warp.on) + ",\"weft_on\":" + String(weft.on) +
      ",\"raw_10s\":" + String(dr) + ",\"min_gap_ms\":" + String(mg == 0xFFFFFFFF ? 0 : mg / 1000) + ",\"rssi\":" + String(WiFi.RSSI()) + ",\"queued\":" + String(qLen) + ",\"up_s\":" + String(now / 1000) + "}";
    Serial.println("SUM " + s);
    if (mqtt.connected()) pub(tSum, s.c_str());
  }
  delay(2);
}
