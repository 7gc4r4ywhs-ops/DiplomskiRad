#include "DisplayManager.h"
#include <Arduino.h>

// Pin definitions for Heltec V3
#define VEXT_PIN 36      // GPIO36 controls power to OLED (LOW = ON)
#define OLED_RST_PIN 21  // GPIO21 is OLED Reset pin

// Create the global instance
DisplayManager Display;

// Constructor definition
DisplayManager::DisplayManager() {
  display = nullptr;
  displayOn = true;
  autoOffTimeout = 0;
  lastActivityTime = 0;
  
  for (int i = 0; i < 4; i++) {
    rowText[i] = "";
  }
  footerText = "";
}

void DisplayManager::init() {
  pinMode(VEXT_PIN, OUTPUT);
  setPower(true);
  
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, LOW);
  delay(50);
  digitalWrite(OLED_RST_PIN, HIGH);
  delay(100);
  
  Wire.begin(17, 18);
  
  display = new SSD1306Wire(0x3c, 17, 18);
  
  if (display->init()) {
    display->flipScreenVertically();
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    displayOn = true;
    clearAll();
    update();
  } else {
    displayOn = false;
  }
  
  resetAutoOffTimer();
}

void DisplayManager::setPower(bool on) {
  if (on) {
    digitalWrite(VEXT_PIN, LOW);
    delay(50);
  } else {
    digitalWrite(VEXT_PIN, HIGH);
  }
}

void DisplayManager::turnOff() {
  if (!displayOn) return;
  
  if (display != nullptr) {
    display->displayOff();
  }
  
  setPower(false);
  displayOn = false;
}

void DisplayManager::turnOn() {
  if (displayOn) return;
  
  setPower(true);
  delay(100);
  
  if (display != nullptr) {
    display->displayOn();
    display->init();
    display->flipScreenVertically();
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    render();
  }
  
  displayOn = true;
  resetAutoOffTimer();
}

bool DisplayManager::isOn() {
  return displayOn;
}

void DisplayManager::setAutoOff(unsigned long timeoutMs) {
  autoOffTimeout = timeoutMs;
  if (autoOffTimeout > 0) {
    resetAutoOffTimer();
  }
}

void DisplayManager::resetAutoOffTimer() {
  lastActivityTime = millis();
}

void DisplayManager::updateAutoOff() {
  if (autoOffTimeout > 0 && displayOn) {
    if (millis() - lastActivityTime >= autoOffTimeout) {
      turnOff();
    }
  }
}

void DisplayManager::clearContent() {
  for (int i = 0; i < 4; i++) {
    rowText[i] = "";
  }
  resetAutoOffTimer();
}

void DisplayManager::clearFooter() {
  footerText = "";
  resetAutoOffTimer();
}

void DisplayManager::clearAll() {
  clearContent();
  clearFooter();
  resetAutoOffTimer();
}

void DisplayManager::setRow(int rowNum, const String& text) {
  if (rowNum >= 1 && rowNum <= 4) {
    rowText[rowNum - 1] = text;
    resetAutoOffTimer();
  }
}

void DisplayManager::setFooter(const String& text) {
  footerText = text;
  resetAutoOffTimer();
}

void DisplayManager::update() {
  if (display != nullptr && displayOn) {
    render();
  }
  resetAutoOffTimer();
}

void DisplayManager::render() {
  if (display == nullptr || !displayOn) return;
  
  display->clear();
  
  display->setFont(ArialMT_Plain_10);
  int rowPositions[4] = {ROW1_Y, ROW2_Y, ROW3_Y, ROW4_Y};
  
  for (int i = 0; i < 4; i++) {
    if (rowText[i].length() > 0) {
      String text = rowText[i];
      if (text.length() > 21) {
        text = text.substring(0, 18) + "...";
      }
      display->drawString(0, rowPositions[i], text);
    }
  }
  
  display->drawLine(0, SEPARATOR_Y, SCREEN_WIDTH, SEPARATOR_Y);
  
  if (footerText.length() > 0) {
    display->setFont(ArialMT_Plain_10);
    
    String footer = footerText;
    if (footer.length() > 21) {
      footer = footer.substring(0, 18) + "...";
    }
    
    display->drawString(0, FOOTER_Y, footer);
  }
  
  display->display();
}

void DisplayManager::showTemporaryMessage(const String& message, int delayMs) {
  if (display == nullptr) return;
  
  String savedFooter = footerText;
  String savedRow[4];
  for (int i = 0; i < 4; i++) {
    savedRow[i] = rowText[i];
  }
  
  bool wasOn = displayOn;
  if (!wasOn) {
    turnOn();
  }
  
  clearContent();
  setRow(2, message);
  update();
  
  delay(delayMs);
  
  footerText = savedFooter;
  for (int i = 0; i < 4; i++) {
    rowText[i] = savedRow[i];
  }
  
  if (!wasOn) {
    turnOff();
  } else {
    update();
  }
  
  resetAutoOffTimer();
}

void DisplayManager::showBootScreen(const String& line1, const String& line2) {
  turnOn();
  clearAll();
  setRow(2, line1);
  setRow(3, line2);
  setFooter("Initializing...");
  update();
  resetAutoOffTimer();
}