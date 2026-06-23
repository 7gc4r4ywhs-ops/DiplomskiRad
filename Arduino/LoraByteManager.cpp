#include "LoRaByteManager.h"
#include "NodeConfig.h"
#include <Arduino.h>

// Synced network clock — defined in main.cpp
extern uint64_t getCurrentTimeMicros();

LoRaByteManager* LoRaByteManager::instance = nullptr;

void LoRaByteManager::setFlag(void) {
  if (instance != nullptr) instance->operationDone = true;
}

LoRaByteManager::LoRaByteManager() {
  ssPin         = SS;
  dio0Pin       = DIO0;
  rstPin        = RST_LoRa;
  busyPin       = BUSY_LoRa;
  radio         = nullptr;
  operationDone = false;
  txInProgress  = false;
  rxActive      = false;
  txTargetIdx   = -1;
  txStartTime   = 0;
  txQueueCount  = 0;
  lastRSSI      = 0;
  lastSNR       = 0;
  instance      = this;

  for (int i = 0; i < TX_QUEUE_SIZE; i++) txQueue[i].active = false;
}

bool LoRaByteManager::begin() {
  Serial.println("[LoRaByte] Initializing...");

  radio = new SX1262(new Module(ssPin, dio0Pin, rstPin, busyPin));
  int state = radio->begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRaByte] Init failed, code %d\n", state);
    return false;
  }

  radio->setFrequency(LORA_FREQ);
  radio->setSpreadingFactor(LORA_SF);
  radio->setBandwidth(LORA_BW);
  radio->setCodingRate(LORA_CR);
  radio->setOutputPower(LORA_POWER);
  radio->setCRC(true);
  radio->setDio1Action(setFlag);

  startRx();
  Serial.println("[LoRaByte] Ready — listening");
  return true;
}

// ── Public: enqueue packet ────────────────────────────────────────────────────

bool LoRaByteManager::sendPacket(const uint8_t* buf, size_t len, uint8_t nodeId, bool isData) {
  if (txQueueCount >= TX_QUEUE_SIZE) {
    Serial.println("[LoRaByte] TX queue full — dropping packet");
    return false;
  }

  for (int i = 0; i < TX_QUEUE_SIZE; i++) {
    if (!txQueue[i].active) {
      memcpy(txQueue[i].buf, buf, len);
      txQueue[i].len       = len;
      txQueue[i].retries   = 0;
      txQueue[i].sendAfter = millis();
      txQueue[i].active    = true;
      txQueue[i].isData    = isData;
      txQueueCount++;
      Serial.printf("[LoRaByte] Enqueued %d bytes, queue=%d/%d\n",
                    len, txQueueCount, TX_QUEUE_SIZE);
      return true;
    }
  }
  return false;
}

// ── Public: update() — call every loop() ─────────────────────────────────────

void LoRaByteManager::update() {
  if (radio == nullptr) return;

  // ── Handle TX complete interrupt ──────────────────────────────────────────
  if (operationDone && txInProgress) {
    operationDone = false;
    txInProgress  = false;
    if (txTargetIdx >= 0 && txTargetIdx < TX_QUEUE_SIZE) {
      txQueue[txTargetIdx].active = false;
      txQueueCount--;
      unsigned long txDuration = millis() - txStartTime;
      Serial.printf("[LoRaByte] TX complete in %lums, queue=%d/%d\n",
                    txDuration, txQueueCount, TX_QUEUE_SIZE);
    }
    txTargetIdx = -1;
    startRx();
  }

  // ── Process TX queue — only when in RX mode and no pending RX packet ──────
  if (!txInProgress && rxActive && !operationDone) {
    processTxQueue();
  }
}

// ── Public: RX ───────────────────────────────────────────────────────────────

bool LoRaByteManager::packetReceived() {
  if (radio == nullptr) return false;
  return (operationDone && !txInProgress);
}

bool LoRaByteManager::readPacket(uint8_t* buf, size_t& len) {
  if (radio == nullptr || !packetReceived()) return false;

  len = radio->getPacketLength();
  int state = radio->readData(buf, len);
  operationDone = false;

  if (state == RADIOLIB_ERR_NONE) {
    lastRSSI = radio->getRSSI();
    lastSNR  = radio->getSNR();
    startRx();
    return true;
  }

  Serial.printf("[LoRaByte] Read failed, code %d\n", state);
  startRx();
  return false;
}

// ── Private helpers ───────────────────────────────────────────────────────────

bool LoRaByteManager::isInTdmaSlot() const {
  uint8_t slot = Node.getTdmaSlot();
  if (slot == 0) return true;  // not assigned — allow anytime

  // Use synced network clock (ms) so all nodes share the same cycle reference
  uint64_t nowMs = getCurrentTimeMicros() / 1000ULL;
  unsigned long cyclePos  = (unsigned long)(nowMs % TDMA_CYCLE_MS);
  unsigned long slotStart = (unsigned long)(slot - 1) * TDMA_SLOT_DURATION_MS;
  unsigned long slotEnd   = slotStart + TDMA_SLOT_DURATION_MS - TDMA_GUARD_MS;

  if (cyclePos < slotStart + TDMA_GUARD_MS) return false;
  if (cyclePos >= slotEnd) return false;

  // Must have at least 2 seconds remaining in slot
  unsigned long remaining = slotEnd - cyclePos;
  return (remaining >= 2800UL);
}

void LoRaByteManager::processTxQueue() {
  int idx = findNextReady();
  if (idx < 0) return;

  // DATA packets must wait for TDMA slot
  if (txQueue[idx].isData && !isInTdmaSlot()) return;

  // Only send one DATA packet per slot entry
  startTx(idx);
}

void LoRaByteManager::startTx(int queueIdx) {
  txInProgress  = true;
  rxActive      = false;
  txTargetIdx   = queueIdx;
  operationDone = false;
  txStartTime   = millis();
  Serial.printf("[LoRaByte] TX %d bytes\n", txQueue[queueIdx].len);
  radio->startTransmit(txQueue[queueIdx].buf, txQueue[queueIdx].len);
}

void LoRaByteManager::startRx() {
  rxActive      = true;
  txInProgress  = false;
  operationDone = false;
  radio->startReceive();
}

int LoRaByteManager::findNextReady() {
  unsigned long now = millis();
  for (int i = 0; i < TX_QUEUE_SIZE; i++) {
    if (txQueue[i].active && now >= txQueue[i].sendAfter) return i;
  }
  return -1;
}

// ── Info ──────────────────────────────────────────────────────────────────────

float LoRaByteManager::getLastRSSI() { return lastRSSI; }
float LoRaByteManager::getLastSNR()  { return lastSNR; }

void LoRaByteManager::printConfig() {
  Serial.println("=== LoRaByte Configuration ===");
  Serial.printf("Frequency:        %.1f MHz\n", LORA_FREQ);
  Serial.printf("Spreading Factor: %d\n",       LORA_SF);
  Serial.printf("Bandwidth:        %.1f kHz\n", LORA_BW);
  Serial.printf("Coding Rate:      4/%d\n",      LORA_CR);
  Serial.printf("Output Power:     %d dBm\n",   LORA_POWER);
  Serial.printf("TX Queue:         %d/%d\n",    txQueueCount, TX_QUEUE_SIZE);
  Serial.println("==============================");
}

LoRaByteManager loraByte;