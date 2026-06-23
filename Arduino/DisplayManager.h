#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Wire.h>
#include "SSD1306Wire.h"

// OLED dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Display area layout
#define ROW1_Y 2
#define ROW2_Y 14
#define ROW3_Y 26
#define ROW4_Y 38
#define FOOTER_Y 52
#define SEPARATOR_Y 50

class DisplayManager {
public:
  // Constructor
  DisplayManager();
  
  // Initialize the display (handles power, reset, I2C)
  void init();
  
  // Clear the main content area (rows 1-4 only)
  void clearContent();
  
  // Clear only the footer
  void clearFooter();
  
  // Clear everything (content and footer)
  void clearAll();
  
  // Set text for a specific row (1-4)
  void setRow(int rowNum, const String& text);
  
  // Set footer text
  void setFooter(const String& text);
  
  // Update the display (call after making changes)
  void update();
  
  // Show a temporary message that disappears after delay ms
  void showTemporaryMessage(const String& message, int delayMs = 2000);
  
  // Show boot screen
  void showBootScreen(const String& line1, const String& line2);
  
  // === POWER CONTROL FUNCTIONS ===
  
  // Turn off the display (cuts power to OLED)
  void turnOff();
  
  // Turn on the display (restores power and refreshes content)
  void turnOn();
  
  // Check if display is currently on
  bool isOn();
  
  // Set auto-off after inactivity (milliseconds, 0 = disabled)
  void setAutoOff(unsigned long timeoutMs);
  
  // Reset auto-off timer (call when activity detected)
  void resetAutoOffTimer();
  
  // Check and handle auto-off (call this in main loop)
  void updateAutoOff();
  
  // Variables needed for auto-off (public so main sketch can access)
  unsigned long autoOffTimeout;
  unsigned long lastActivityTime;

private:
  SSD1306Wire* display;
  
  // Storage for each area
  String rowText[4];  // rows 1-4
  String footerText;
  
  // Power state
  bool displayOn;
  
  // Internal method to render everything
  void render();
  
  // Internal method to physically power the display
  void setPower(bool on);
};

// Global instance
extern DisplayManager Display;

#endif