#ifndef LORA_BYTE_MANAGER_H
#define LORA_BYTE_MANAGER_H

#include <RadioLib.h>
#include <pins_arduino.h>

#define LORA_FREQ   868.0
#define LORA_SF     12
#define LORA_BW     125.0
#define LORA_CR     8
#define LORA_POWER  22

#define LORA_MAX_PACKET_SIZE 255

// TDMA settings
#define TDMA_SLOT_DURATION_MS  3500UL  // 3.5 seconds per slot
#define TDMA_CYCLE_MS          10500UL // 10.5 seconds total cycle (3 slots)
#define TDMA_GUARD_MS          100UL   // guard time at start of slot

// TX queue settings
#define TX_QUEUE_SIZE    8

struct TxQueueEntry {
  uint8_t       buf[LORA_MAX_PACKET_SIZE];
  size_t        len;
  uint8_t       retries;
  unsigned long sendAfter;
  bool          active;
  bool          isData;  // true = DATA packet, enforce TDMA slot
};

class LoRaByteManager {
public:
  LoRaByteManager();

  bool begin();

  // Enqueue packet for TX — non-blocking, returns false if queue full
  // nodeId used for ID-based backoff on DATA packets (set via sendDataPacket)
  // isData = true enforces TDMA slot timing
  bool sendPacket(const uint8_t* buf, size_t len, uint8_t nodeId = 0, bool isData = false);

  // Must be called every loop()
  void update();

  // Returns true when a received packet is waiting to be read
  bool packetReceived();

  // Reads received bytes into buf, sets len to actual received length
  bool readPacket(uint8_t* buf, size_t& len);

  float getLastRSSI();
  float getLastSNR();
  void  printConfig();
  uint8_t getTxQueueCount() const { return txQueueCount; }

private:
  SX1262* radio;
  int ssPin, dio0Pin, rstPin, busyPin;

  volatile bool operationDone;
  static void setFlag(void);
  static LoRaByteManager* instance;

  bool txInProgress;
  bool rxActive;
  int  txTargetIdx;
  unsigned long txStartTime;

  float lastRSSI;
  float lastSNR;

  TxQueueEntry txQueue[TX_QUEUE_SIZE];
  uint8_t      txQueueCount;

  void processTxQueue();
  void startTx(int queueIdx);
  void startRx();
  int  findNextReady();
  bool isInTdmaSlot() const;
};

extern LoRaByteManager loraByte;

#endif