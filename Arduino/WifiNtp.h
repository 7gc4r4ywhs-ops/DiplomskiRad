#ifndef WIFI_NTP_H
#define WIFI_NTP_H

#include <Arduino.h>
#include <Preferences.h>

// Default NTP server
#define NTP_SERVER "pool.ntp.org"

// Default timezone — Croatia (CET/CEST with automatic DST)
#define NTP_DEFAULT_TZ "CET-1CEST,M3.5.0,M10.5.0/3"

// Retry interval if WiFi connection fails (ms)
#define WIFI_RETRY_INTERVAL_MS 30000

// Timeout waiting for WiFi connection (ms)
#define WIFI_CONNECT_TIMEOUT_MS 10000

// Timeout waiting for NTP sync (ms)
#define NTP_SYNC_TIMEOUT_MS 10000

class WifiNtp {
public:
  WifiNtp();

  // Load credentials from flash. Call in setup() on master.
  void begin();

  // Called every loop() — drives connection and NTP sync state machine.
  // Returns true once NTP sync is complete.
  bool update();

  // True if WiFi is currently connected
  bool isConnected() const;

  // True if NTP time has been successfully synced at least once
  bool isSynced() const;

  // Returns current Unix timestamp in seconds (0 if not synced)
  uint32_t getUnixTime() const;

  // Credential setters — persist to flash
  void setSSID(const String& ssid);
  void setPassword(const String& password);
  void setTimezone(const String& tz);

  // Getters
  String getSSID()     const;
  String getTimezone() const;

  // Print config to Serial (password hidden)
  void printConfig() const;

  // Force a re-sync (resets internal state)
  void resetSync();

private:
  Preferences prefs;

  String ssid;
  String password;
  String timezone;

  bool connected;
  bool synced;

  unsigned long lastRetryTime;
  unsigned long connectStartTime;
  unsigned long syncStartTime;

  enum class WifiState : uint8_t {
    IDLE,           // No credentials set
    CONNECTING,     // WiFi.begin() called, waiting
    CONNECTED,      // WiFi connected, starting NTP
    SYNCING,        // Waiting for NTP to set time
    DONE,           // Synced and staying connected
    RETRY_WAIT,     // Connection failed, waiting before retry
  };
  WifiState state;

  void startConnection();
  void loadFromFlash();
};

extern WifiNtp WifiNtpManager;

#endif