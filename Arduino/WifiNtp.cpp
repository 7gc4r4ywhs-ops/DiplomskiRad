#include "WifiNtp.h"
#include "DisplayManager.h"
#include <WiFi.h>
#include <time.h>
#include <esp_timer.h>

WifiNtp WifiNtpManager;

WifiNtp::WifiNtp() {
  ssid            = "";
  password        = "";
  timezone        = NTP_DEFAULT_TZ;
  connected       = false;
  synced          = false;
  lastRetryTime   = 0;
  connectStartTime = 0;
  syncStartTime   = 0;
  state           = WifiState::IDLE;
}

void WifiNtp::begin() {
  loadFromFlash();
  Serial.println("[WiFiNTP] Initialized");
  printConfig();
}

void WifiNtp::loadFromFlash() {
  prefs.begin("ntp", true);
  ssid     = prefs.getString("ssid",     "");
  password = prefs.getString("password", "");
  timezone = prefs.getString("timezone", NTP_DEFAULT_TZ);
  prefs.end();
}

void WifiNtp::setSSID(const String& s) {
  ssid = s;
  prefs.begin("ntp", false);
  prefs.putString("ssid", s);
  prefs.end();
  Serial.printf("[WiFiNTP] SSID set to: %s\n", s.c_str());
}

void WifiNtp::setPassword(const String& p) {
  password = p;
  prefs.begin("ntp", false);
  prefs.putString("password", p);
  prefs.end();
  Serial.println("[WiFiNTP] Password saved");
}

void WifiNtp::setTimezone(const String& tz) {
  timezone = tz;
  prefs.begin("ntp", false);
  prefs.putString("timezone", tz);
  prefs.end();
  Serial.printf("[WiFiNTP] Timezone set to: %s\n", tz.c_str());
  // Apply immediately if already synced
  if (synced) setenv("TZ", timezone.c_str(), 1);
}

String WifiNtp::getSSID()     const { return ssid; }
String WifiNtp::getTimezone() const { return timezone; }
bool   WifiNtp::isConnected() const { return connected; }
bool   WifiNtp::isSynced()    const { return synced; }

uint32_t WifiNtp::getUnixTime() const {
  if (!synced) return 0;
  return (uint32_t)time(nullptr);
}

void WifiNtp::resetSync() {
  synced    = false;
  connected = false;
  state     = WifiState::IDLE;
  WiFi.disconnect();
  Serial.println("[WiFiNTP] Sync reset");
}

void WifiNtp::printConfig() const {
  Serial.println("=== WiFi / NTP Config ===");
  Serial.printf("SSID:     %s\n", ssid.length() > 0 ? ssid.c_str() : "NOT SET");
  Serial.printf("Password: %s\n", password.length() > 0 ? "****" : "NOT SET");
  Serial.printf("Timezone: %s\n", timezone.c_str());
  Serial.printf("NTP:      %s\n", NTP_SERVER);
  Serial.println("=========================");
}

// ── Internal ──────────────────────────────────────────────────────────────────

void WifiNtp::startConnection() {
  Serial.printf("[WiFiNTP] Connecting to: %s\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  connectStartTime = millis();
  state = WifiState::CONNECTING;

  Display.clearAll();
  Display.setRow(1, "NTP SYNC");
  Display.setRow(2, "WiFi: " + ssid);
  Display.setRow(3, "Connecting...");
  Display.update();
}

// ── update() — call every loop() ─────────────────────────────────────────────

bool WifiNtp::update() {
  switch (state) {

    case WifiState::IDLE:
      if (ssid.length() == 0) {
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 5000) {
          Serial.println("[WiFiNTP] No SSID set — use: setssid <ssid> and setwifipass <password>");
          lastLog = millis();
        }
        Display.clearAll();
        Display.setRow(1, "NTP SYNC");
        Display.setRow(2, "No WiFi config!");
        Display.setRow(3, "setwifi <s> <p>");
        Display.update();
        return false;
      }
      startConnection();
      break;

    case WifiState::CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.printf("[WiFiNTP] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

        // Configure NTP
        configTzTime(timezone.c_str(), NTP_SERVER);
        syncStartTime = millis();
        state = WifiState::SYNCING;

        Display.clearAll();
        Display.setRow(1, "NTP SYNC");
        Display.setRow(2, "WiFi: OK");
        Display.setRow(3, WiFi.localIP().toString());
        Display.setRow(4, "Syncing time...");
        Display.update();

      } else if (millis() - connectStartTime >= WIFI_CONNECT_TIMEOUT_MS) {
        Serial.println("[WiFiNTP] WiFi connection timeout — retrying in 30s");
        WiFi.disconnect();
        connected       = false;
        lastRetryTime   = millis();
        state           = WifiState::RETRY_WAIT;

        Display.clearAll();
        Display.setRow(1, "NTP SYNC");
        Display.setRow(2, "WiFi FAILED");
        Display.setRow(3, "Retry in 30s");
        Display.update();
      }
      break;

    case WifiState::SYNCING: {
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        synced = true;
        state  = WifiState::DONE;

        // Set clockOffset so esp_timer_get_time() + clockOffset = Unix time in us
        // This allows peer-to-peer sync to propagate correct absolute time
        extern int64_t clockOffset;
        time_t unixNow = mktime(&timeinfo);
        int64_t unixUs = (int64_t)unixNow * 1000000LL;
        clockOffset = unixUs - (int64_t)esp_timer_get_time();
        Serial.printf("[WiFiNTP] clockOffset set to %lld us\n", clockOffset);

        char timeBuf[20];
        strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &timeinfo);
        char dateBuf[20];
        strftime(dateBuf, sizeof(dateBuf), "%d.%m.%Y", &timeinfo);

        Serial.printf("[WiFiNTP] Time synced: %s %s\n", dateBuf, timeBuf);

        Display.clearAll();
        Display.setRow(1, "NTP SYNC: OK");
        Display.setRow(2, String(dateBuf));
        Display.setRow(3, String(timeBuf));
        Display.setRow(4, timezone);
        Display.update();

        return true;  // Signal sync complete

      } else if (millis() - syncStartTime >= NTP_SYNC_TIMEOUT_MS) {
        Serial.println("[WiFiNTP] NTP sync timeout — retrying in 30s");
        lastRetryTime = millis();
        state         = WifiState::RETRY_WAIT;

        Display.clearAll();
        Display.setRow(1, "NTP SYNC");
        Display.setRow(2, "NTP FAILED");
        Display.setRow(3, "Retry in 30s");
        Display.update();
      }
      break;
    }

    case WifiState::DONE:
      // Stay connected, check if WiFi dropped
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFiNTP] WiFi dropped — reconnecting");
        connected = false;
        startConnection();
      }
      // Already synced — return true every update to keep SM happy
      return synced;

    case WifiState::RETRY_WAIT:
      if (millis() - lastRetryTime >= WIFI_RETRY_INTERVAL_MS) {
        Serial.println("[WiFiNTP] Retrying WiFi connection...");
        startConnection();
      }
      break;
  }

  return false;
}