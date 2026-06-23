#include "LoRaByteManager.h"
#include "DisplayManager.h"
#include "NodeConfig.h"
#include "LoRaPacket.h"
#include "StateMachine.h"
#include "NetworkTopology.h"
#include "WifiNtp.h"
#include <Preferences.h>
#include <esp_timer.h>

#define FW_VERSION "v1.0"

// Serial buffer
String serialBuffer = "";

// TDMA propagation — delayed 10s after topology change
bool          tdmaPropagationPending = false;
unsigned long tdmaPropagationAt      = 0;

// Receive / transmit buffers
uint8_t rxBuf[PKT_TOTAL_LEN];
uint8_t txBuf[PKT_TOTAL_LEN];

// Clock offset in microseconds — applied on top of esp_timer_get_time()
int64_t clockOffset = 0;

// Returns current time in microseconds with offset applied
uint64_t getCurrentTimeMicros() {
  return (uint64_t)((int64_t)esp_timer_get_time() + clockOffset);
}

// PRG button — GPIO0, active LOW
// Forward declarations
void onSinglePress();
void onDoublePress();
void onLongPress();
void showClockDisplay();
void sendDataPacket(uint8_t destination, const DataPayload& data);
void sendRemoveInit(uint8_t targetId);
void sendTdmaSlot(uint8_t destination, uint8_t nextHop, uint8_t slot);
void propagateTdmaFromMaster();

#define BTN_PRG_PIN       0
#define DEBOUNCE_MS       50
#define LONG_PRESS_MS     1000
#define DOUBLE_PRESS_MS   500

bool lastBtnState         = HIGH;
bool stableBtnState       = HIGH;
unsigned long lastDebounceTime  = 0;
unsigned long pressStartTime    = 0;   // when button went LOW
unsigned long lastReleaseTime   = 0;   // when button last went HIGH
bool          pressHandled      = false; // long press already fired
uint8_t       pendingClicks     = 0;    // clicks waiting to be resolved

// Display mode in EXPLOITATION
bool clockDisplayActive   = false;
unsigned long lastClockUpdate = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Display.init();
  Display.showBootScreen("LoRa Node " FW_VERSION, "Starting...");

  Serial.println("\n========================================");
  Serial.println("=== LoRa Mesh Node " FW_VERSION " ===");
  Serial.println("========================================\n");

  pinMode(BTN_PRG_PIN, INPUT_PULLUP);

  Node.begin();

  if (Node.isMaster()) {
    Topology.begin();
    WifiNtpManager.begin();
    if (Node.getTdmaSlot() != 1) Node.setTdmaSlot(1);
  }

  SM.flags.enrollmentDataFound = Node.isEnrolled();
  SM.begin(FW_VERSION);

  if (loraByte.begin()) {
    Serial.println("[LoRa] Ready");
    loraByte.printConfig();
  } else {
    Serial.println("[LoRa] Failed!");
    Display.showBootScreen("LoRa", "INIT FAILED");
    while(1);
  }

  Serial.println("\nCommands:");
  Serial.println("  sdata <dst> <km>     - Send DATA packet");
  Serial.println("  spacket <dst> <msg>  - Send MANAGEMENT packet");
  Serial.println("  removenode <id>      - Initiate node removal (master only)");
  Serial.println("  setid <id>           - Set node ID (0=master)");
  Serial.println("  setup <id>           - Set upstream neighbor");
  Serial.println("  setdown <id>         - Set downstream neighbor");
  Serial.println("  setenrolled <0|1>    - Set enrollment flag");
  Serial.println("  config               - Show node config");
  Serial.println("  topology             - Show network topology (master only)");
  Serial.println("  setssid <ssid>       - Set WiFi SSID (master only)");
  Serial.println("  setwifipass <pass>   - Set WiFi password (master only)");
  Serial.println("  showwifi             - Show WiFi config (master only)");
  Serial.println("  settz <tz>           - Set timezone POSIX string (master only)");
  Serial.println("  syncclock            - Sync this node with upstream (enrollment type)");
  Serial.println("  syncnetwork          - Trigger network-wide periodic sync (master only)");
  Serial.println("  syncclock            - Sync this node with upstream");
  Serial.println("  syncnetwork          - Network-wide periodic sync (master only)");
  Serial.println("  sendslots            - Propagate TDMA slots through network (master only)");
  Serial.println("  factoryreset         - Clear all flash data and reboot");
  Serial.println("  help                 - This help");
  Serial.print("> ");
}

void loop() {
  // ── PRG button (single / double / long press) ────────────────────────────
  bool reading = digitalRead(BTN_PRG_PIN);

  if (reading != lastBtnState) {
    lastDebounceTime = millis();
    lastBtnState = reading;
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (stableBtnState == HIGH && reading == LOW) {
      // Falling edge — button pressed
      pressStartTime = millis();
      pressHandled   = false;
    }
    if (stableBtnState == LOW && reading == HIGH) {
      // Rising edge — button released
      unsigned long pressDuration = millis() - pressStartTime;
      if (!pressHandled) {
        if (pressDuration >= LONG_PRESS_MS) {
          onLongPress();
          pressHandled = true;
        } else {
          pendingClicks++;
          lastReleaseTime = millis();
        }
      }
    }
    // Long press
    if (reading == LOW && !pressHandled && (millis() - pressStartTime >= LONG_PRESS_MS)) {
      onLongPress();
      pressHandled = true;
    }
    stableBtnState = reading;
  }

  // Resolve pending clicks after double-press window expires
  if (pendingClicks > 0 && reading == HIGH &&
      (millis() - lastReleaseTime) > DOUBLE_PRESS_MS) {
    if (pendingClicks >= 2) onDoublePress();
    else                    onSinglePress();
    pendingClicks = 0;
  }

  // Update clock display if active in EXPLOITATION
  if (clockDisplayActive && SM.getState() == NodeState::EXPLOITATION) {
    if (millis() - lastClockUpdate >= 1000) {
      showClockDisplay();
      lastClockUpdate = millis();
    }
  }

  // ── Serial commands ───────────────────────────────────────────────────────
  while (Serial.available() > 0) {
    char c = Serial.read();
    serialBuffer += c;
    Serial.print(c);
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        serialBuffer.trim();
        if (serialBuffer.length() > 0) processCommand(serialBuffer);
        serialBuffer = "";
        Serial.print("> ");
      }
    }
  }

  // ── Incoming LoRa packets ─────────────────────────────────────────────────
  if (loraByte.packetReceived()) {
    size_t len = 0;
    if (loraByte.readPacket(rxBuf, len)) handleIncomingPacket(rxBuf, len);
  }

  // TDMA propagation — delayed 10s after topology change
  if (tdmaPropagationPending && millis() >= tdmaPropagationAt) {
    tdmaPropagationPending = false;
    Serial.println("[TDMA] Propagating slots through network");
    propagateTdmaFromMaster();
  }

  Display.updateAutoOff();
  loraByte.update();
  SM.update();
  delay(10);
}

// ── Button handlers ───────────────────────────────────────────────────────────

void showClockDisplay() {
  struct tm timeinfo;

  // Debug — remove after testing
  Serial.printf("[CLOCK] esp_timer:           %llu us\n", esp_timer_get_time());
  Serial.printf("[CLOCK] clockOffset:          %lld us\n", clockOffset);
  Serial.printf("[CLOCK] getCurrentTimeMicros: %llu us\n", getCurrentTimeMicros());
  Serial.printf("[CLOCK] Unix seconds:         %lu\n", (unsigned long)(getCurrentTimeMicros() / 1000000ULL));
  bool gotTime = getLocalTime(&timeinfo);
  Serial.printf("[CLOCK] getLocalTime result:  %d\n", gotTime);

  Display.clearAll();
  Display.setRow(1, Node.isMaster() ? "MASTER" : "Node " + String(Node.getNodeId()));
  if (gotTime) {
    char timeBuf[10], dateBuf[12];
    strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &timeinfo);
    strftime(dateBuf, sizeof(dateBuf), "%d.%m.%Y", &timeinfo);
    Display.setRow(2, String(timeBuf));
    Display.setRow(3, String(dateBuf));
  } else {
    Display.setRow(2, "Time not synced");
    Display.setRow(3, "Unix: " + String((unsigned long)(getCurrentTimeMicros() / 1000000ULL)));
  }
  Display.setFooter(String(FW_VERSION) + " | CLOCK");
  Display.update();
}

void onSinglePress() {
  uint8_t myId = Node.getNodeId();
  Serial.println("[BTN] Single press");

  if (Node.isMaster()) {
    Display.showTemporaryMessage("I am master", 2000);
    return;
  }

  if (SM.getState() == NodeState::INITIAL) {
    if (myId == NODE_ID_UNSET) {
      Display.showTemporaryMessage("Set node ID first", 2000);
      return;
    }
    if (Node.getUpstream() == NODE_NEIGHBOR_NONE) {
      Display.showTemporaryMessage("Set upstream first", 2000);
      return;
    }
    Serial.println("[BTN] Starting enrollment");
    SM.flags.enrollmentRequested = true;
    return;
  }

  if (SM.getState() == NodeState::EXPLOITATION) {
    if (clockDisplayActive) {
      // Single press while clock is showing — do nothing (long press toggles)
      return;
    }
    DataPayload data;
    data.distance  = 0;
    data.timestamp = (uint32_t)(getCurrentTimeMicros() / 1000000ULL);
    memset(data.padding, 0x00, sizeof(data.padding));
    Serial.println("\n[BTN] Sending test DATA packet to master");
    sendDataPacket(NODE_ID_MASTER, data);
    return;
  }

  Serial.printf("[BTN] Ignored in state: %s\n", SM.getStateName());
}

void onDoublePress() {
  Serial.println("[BTN] Double press");

  if (SM.getState() == NodeState::EXPLOITATION) {
    clockDisplayActive = false;
    // Return to default exploitation display
    Display.clearAll();
    Display.setRow(1, Node.isMaster() ? "MASTER" : "Node " + String(Node.getNodeId()));
    Display.setRow(2, SM.getStateName());
    Display.setFooter(String(FW_VERSION) + " | " + SM.getStateName());
    Display.update();
    Serial.println("[BTN] Returned to default display");
    return;
  }

  Serial.printf("[BTN] Double press ignored in state: %s\n", SM.getStateName());
}

void onLongPress() {
  Serial.println("[BTN] Long press");

  if (SM.getState() == NodeState::EXPLOITATION) {
    clockDisplayActive = !clockDisplayActive;
    if (clockDisplayActive) {
      Serial.println("[BTN] Clock display ON");
      showClockDisplay();
      lastClockUpdate = millis();
    } else {
      Serial.println("[BTN] Clock display OFF");
      Display.clearAll();
      Display.setRow(1, Node.isMaster() ? "MASTER" : "Node " + String(Node.getNodeId()));
      Display.setRow(2, SM.getStateName());
      Display.setFooter(String(FW_VERSION) + " | " + SM.getStateName());
      Display.update();
    }
    return;
  }

  Serial.printf("[BTN] Long press ignored in state: %s\n", SM.getStateName());
}

// ── Routing validation ────────────────────────────────────────────────────────

bool validateDestination(uint8_t destination) {
  uint8_t myId = Node.getNodeId();
  if (myId == NODE_ID_UNSET) { Serial.println("[ERROR] Node ID not set"); return false; }
  if (!Node.isMaster()) {
    bool ok = (destination == NODE_ID_MASTER)      ||
              (destination == Node.getUpstream())   ||
              (destination == Node.getDownstream());
    if (!ok) {
      Serial.printf("[ERROR] Node %d can only send to master, upstream (%d) or downstream (%d)\n",
                    myId, Node.getUpstream(), Node.getDownstream());
      Display.showTemporaryMessage("DST not allowed", 2000);
      return false;
    }
  }
  return true;
}

// ── Packet send ───────────────────────────────────────────────────────────────

void sendDataPacket(uint8_t destination, const DataPayload& data) {
  if (!validateDestination(destination)) return;
  uint8_t myId    = Node.getNodeId();
  uint8_t nextHop = Node.resolveNextHop(destination);
  if (nextHop == NODE_NEIGHBOR_NONE) { Serial.println("[ERROR] Cannot resolve next hop"); return; }

  LoRaPacket pkt;
  buildDataPacket(pkt, myId, destination, myId, nextHop, data, txBuf);
  Serial.println("\n[SEND] DATA packet:"); printPacket(pkt);

  bool success = loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId(), true);
  Serial.printf("[SEND] %s\n", success ? "Success" : "Failed");

  Display.clearAll();
  Display.setRow(1, "TX: DATA");
  Display.setRow(2, "Dist: " + String(data.distance) + " km");
  Display.setRow(3, success ? "Status: OK" : "Status: FAIL");
  Display.setRow(4, "DST: " + String(destination));
  Display.setFooter(String(FW_VERSION) + " | TX DATA");
  Display.update();
}

void sendManagementPacket(uint8_t destination, const ManagementPayload& mgmt) {
  if (!validateDestination(destination)) return;
  uint8_t myId    = Node.getNodeId();
  uint8_t nextHop = Node.resolveNextHop(destination);
  if (nextHop == NODE_NEIGHBOR_NONE) { Serial.println("[ERROR] Cannot resolve next hop"); return; }

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, destination, myId, nextHop, mgmt, txBuf);
  Serial.println("\n[SEND] MANAGEMENT packet:"); printPacket(pkt);

  bool success = loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
  Serial.printf("[SEND] %s\n", success ? "Success" : "Failed");

  Display.clearAll();
  Display.setRow(1, "TX: MGMT");
  Display.setRow(2, "Sub: 0x" + String(mgmt.subtype, HEX));
  Display.setRow(3, success ? "Status: OK" : "Status: FAIL");
  Display.setRow(4, "DST: " + String(destination));
  Display.setFooter(String(FW_VERSION) + " | TX MGMT");
  Display.update();
}

// ── Master: initiate node removal ─────────────────────────────────────────────

void initiateNodeRemoval(uint8_t targetId) {
  if (!Node.isMaster()) {
    Serial.println("[ERROR] Only master can initiate removal");
    return;
  }
  if (targetId == NODE_ID_MASTER) {
    Serial.println("[ERROR] Cannot remove master");
    return;
  }

  uint8_t myId    = Node.getNodeId();
  uint8_t nextHop = Node.resolveNextHop(targetId);
  if (nextHop == NODE_NEIGHBOR_NONE) {
    Serial.println("[ERROR] Cannot resolve next hop for removal target");
    return;
  }

  // Check no other network action is running
  if (SM.networkSyncInProgress || SM.networkRemovalInProgress) {
    Serial.println("[ERROR] Network action in progress — cannot start removal now");
    Display.showTemporaryMessage("Action in progress!", 2000);
    return;
  }
  SM.networkRemovalInProgress = true;
  SM.flags.removalInitSent    = false;

  // Store target ID and its topology entry for later cleanup
  SM.flags.removalTargetId = targetId;
  NodeEntry targetEntry;
  if (Topology.getNode(targetId, targetEntry)) {
    SM.flags.removalOrigUpstream   = targetEntry.upstreamId;
    SM.flags.removalOrigDownstream = targetEntry.downstreamId;
  } else {
    SM.flags.removalOrigUpstream   = NODE_NEIGHBOR_NONE;
    SM.flags.removalOrigDownstream = NODE_NEIGHBOR_NONE;
  }

  Serial.printf("[MASTER] Node %d scheduled for removal\n", targetId);
  Display.clearAll();
  Display.setRow(1, "REMOVAL QUEUED");
  Display.setRow(2, "Target: " + String(targetId));
  Display.setFooter(String(FW_VERSION) + " | REMOVAL");
  Display.update();
}

void sendRemoveInit(uint8_t targetId) {
  uint8_t myId    = Node.getNodeId();
  uint8_t nextHop = Node.resolveNextHop(targetId);
  if (nextHop == NODE_NEIGHBOR_NONE) { Serial.println("[ERROR] Cannot resolve next hop"); return; }

  RemoveInitPayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype  = MGMT_REMOVE_INIT;
  p.targetId = targetId;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(RemoveInitPayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, targetId, myId, nextHop, mgmt, txBuf);
  Serial.printf("[MASTER] Sending REMOVE_INIT to node %d\n", targetId);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());

  Display.clearAll();
  Display.setRow(1, "REMOVAL INIT");
  Display.setRow(2, "Target: " + String(targetId));
  Display.setRow(3, "In progress...");
  Display.setFooter(String(FW_VERSION) + " | REMOVAL");
  Display.update();
}

// ── NODE_ADD_ACK sender ───────────────────────────────────────────────────────

void sendTdmaSlot(uint8_t destination, uint8_t nextHop, uint8_t slot) {
  uint8_t myId = Node.getNodeId();

  TdmaSlotPayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype = MGMT_TDMA_SLOT;
  p.slot    = slot;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(TdmaSlotPayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, destination, myId, nextHop, mgmt, txBuf);
  Serial.printf("[TDMA] Sending slot %d to node %d\n", slot, destination);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, myId);
}

void propagateTdmaFromMaster() {
  // Master is slot 1 — send slot 2 to its downstream
  uint8_t downstream = Node.getDownstream();
  if (downstream == NODE_NEIGHBOR_NONE) return;
  sendTdmaSlot(downstream, downstream, 2);
}

void sendNodeAddAck(uint8_t destination) {
  uint8_t myId = Node.getNodeId();

  NodeAddAckPayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype    = MGMT_NODE_ADD_ACK;
  p.neighborId = myId;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(NodeAddAckPayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, destination, myId, destination, mgmt, txBuf);
  Serial.printf("[ENROLL] Sending NODE_ADD_ACK to %d\n", destination);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

// ── Packet receive & forward ──────────────────────────────────────────────────

void handleIncomingPacket(const uint8_t* buf, size_t len) {
  LoRaPacket pkt;
  if (!parsePacket(buf, len, pkt)) {
    Serial.println("[RX] Dropped: CRC error");
    Display.showTemporaryMessage("RX: CRC ERROR", 2000);
    return;
  }

  uint8_t myId = Node.getNodeId();

  // ── Deliver ───────────────────────────────────────────────────────────────
  if (pkt.destination == myId && pkt.nextHop == myId) {
    Serial.println("\n[RX] Packet delivered:"); printPacket(pkt);
    switch (pkt.msgType) {
      case PKT_TYPE_DATA:       handleDataPacket(pkt);       break;
      case PKT_TYPE_MANAGEMENT: handleManagementPacket(pkt); break;
      case PKT_TYPE_TIMESYNC:   handleTimeSyncPacket(pkt);   break;
      default: Serial.printf("[RX] Unknown type: 0x%02X\n", pkt.msgType); break;
    }
    return;
  }

  // ── Ignore ────────────────────────────────────────────────────────────────
  if (pkt.nextHop != myId) {
    Serial.printf("[RX] Ignored (dst=%d, nextHop=%d)\n", pkt.destination, pkt.nextHop);
    return;
  }

  // ── TTL check ─────────────────────────────────────────────────────────────
  if (pkt.ttl <= PKT_TTL_EXPIRED) {
    Serial.println("[RX] Dropped: TTL expired");
    Display.showTemporaryMessage("RX: TTL EXPIRED", 2000);
    return;
  }

  // ── Forward ───────────────────────────────────────────────────────────────
  uint8_t forwardTo = Node.resolveNextHop(pkt.destination);
  if (forwardTo == NODE_NEIGHBOR_NONE) {
    Serial.println("[RX] Dropped: no route");
    Display.showTemporaryMessage("RX: NO ROUTE", 2000);
    return;
  }

  pkt.prevHop = myId;
  pkt.nextHop = forwardTo;
  pkt.ttl--;
  pkt.crc = computeCRC16(pkt.payload, PKT_PAYLOAD_LEN);
  serializePacket(pkt, txBuf);

  Serial.println("\n[FWD] Forwarding:"); printPacket(pkt);
  bool isDataFwd = (pkt.msgType == PKT_TYPE_DATA);
  bool success = loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId(), isDataFwd);
  Serial.printf("[FWD] %s (TTL: %d)\n", success ? "OK" : "FAIL", pkt.ttl);

  Display.clearAll();
  Display.setRow(1, "S:" + String(pkt.source) + " D:" + String(pkt.destination));
  Display.setRow(2, "PH:" + String(pkt.prevHop) + " NH:" + String(pkt.nextHop));
  Display.setRow(3, "FWD -> " + String(forwardTo));
  Display.setRow(4, "TTL: " + String(pkt.ttl));
  Display.setFooter(String(FW_VERSION) + " | FWD");
  Display.update();
}

// ── Message type handlers ─────────────────────────────────────────────────────

void handleDataPacket(const LoRaPacket& pkt) {
  DataPayload data; extractDataPayload(pkt, data);

  // Format timestamp as HH:MM:SS
  char timeBuf[10] = "??:??:??";
  time_t t = (time_t)data.timestamp;
  struct tm* ti = localtime(&t);
  if (ti) strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", ti);

  Serial.printf("[DATA] Dist: %d km, Time: %s (%u), From: %d\n",
                data.distance, timeBuf, data.timestamp, pkt.source);

  Display.clearAll();
  Display.setRow(1, "RX: DATA");
  Display.setRow(2, "Dist: " + String(data.distance) + " km");
  Display.setRow(3, String(timeBuf));
  Display.setRow(4, "From: " + String(pkt.source));
  Display.setFooter(String(FW_VERSION) + " | RX DATA");
  Display.update();
}

void handleManagementPacket(const LoRaPacket& pkt) {
  ManagementPayload mgmt; extractManagementPayload(pkt, mgmt);

  switch (mgmt.subtype) {

    // ── Enrollment ──────────────────────────────────────────────────────────
    case MGMT_NODE_ADD: {
      NodeAddPayload p; extractNodeAddPayload(pkt, p);
      Serial.printf("[ENROLL] NODE_ADD from %d, they are our new %s\n",
                    p.nodeId, p.role == ROLE_UPSTREAM ? "UPSTREAM" : "DOWNSTREAM");
      if (p.role == ROLE_UPSTREAM) {
        if (Node.isMaster()) Serial.println("[ENROLL] WARNING: Master ignoring upstream assignment");
        else Node.setUpstream(p.nodeId);
      } else {
        Node.setDownstream(p.nodeId);
      }
      sendNodeAddAck(pkt.source);
      Display.clearAll();
      Display.setRow(1, "RX: NODE_ADD");
      Display.setRow(2, "From: " + String(p.nodeId));
      Display.setRow(3, p.role == ROLE_UPSTREAM ? "New upstream" : "New downstream");
      Display.setRow(4, "ACK sent");
      Display.setFooter(String(FW_VERSION) + " | ENROLL");
      Display.update();
      break;
    }

    case MGMT_NODE_ADD_ACK: {
      NodeAddAckPayload p; extractNodeAddAckPayload(pkt, p);
      Serial.printf("[ENROLL] NODE_ADD_ACK from %d\n", p.neighborId);
      if      (p.neighborId == Node.getUpstream())   SM.flags.upstreamAcked   = true;
      else if (p.neighborId == Node.getDownstream())  SM.flags.downstreamAcked = true;
      else Serial.printf("[ENROLL] WARNING: ACK from unknown neighbor %d\n", p.neighborId);
      Display.clearAll();
      Display.setRow(1, "RX: ACK");
      Display.setRow(2, "From: " + String(p.neighborId));
      Display.setFooter(String(FW_VERSION) + " | ENROLL");
      Display.update();
      break;
    }

    case MGMT_NODE_ADDED: {
      if (!Node.isMaster()) { Serial.println("[MGMT] NODE_ADDED on non-master — ignoring"); return; }
      NodeAddedPayload p; extractNodeAddedPayload(pkt, p);
      Serial.printf("[MASTER] Node %d enrolled (up=%d, down=%d)\n",
                    p.nodeId, p.upstreamId, p.downstreamId);
      Topology.addOrUpdateNode(p.nodeId, p.upstreamId, p.downstreamId);

      // Update neighbors' topology entries
      if (p.upstreamId != NODE_ID_MASTER && p.upstreamId != TOPOLOGY_NODE_NONE) {
        NodeEntry upEntry;
        if (Topology.getNode(p.upstreamId, upEntry))
          Topology.addOrUpdateNode(upEntry.nodeId, upEntry.upstreamId, p.nodeId);
      }
      if (p.downstreamId != TOPOLOGY_NODE_NONE) {
        NodeEntry downEntry;
        if (Topology.getNode(p.downstreamId, downEntry))
          Topology.addOrUpdateNode(downEntry.nodeId, p.nodeId, downEntry.downstreamId);
      }
      if (p.upstreamId == NODE_ID_MASTER) {
        Node.setDownstream(p.nodeId);
        Serial.printf("[MASTER] Updated own downstream to %d\n", p.nodeId);
      }
      Topology.printTopology();
      // Propagate TDMA slots through network
      // Schedule TDMA propagation 10s from now
      tdmaPropagationPending = true;
      tdmaPropagationAt      = millis() + 10000UL;
      Serial.println("[TDMA] Propagation scheduled in 10s");
      Display.clearAll();
      Display.setRow(1, "Node enrolled!");
      Display.setRow(2, "ID: " + String(p.nodeId));
      Display.setRow(3, "UP: " + String(p.upstreamId));
      Display.setRow(4, "DN: " + (p.downstreamId == NODE_NEIGHBOR_NONE ? "none" : String(p.downstreamId)));
      Display.setFooter("Nodes: " + String(Topology.getNodeCount()));
      Display.update();
      break;
    }

    // ── Removal ─────────────────────────────────────────────────────────────
    case MGMT_REMOVE_INIT: {
      Serial.println("[REMOVE] REMOVE_INIT received — transitioning to REMOVE_SELF");
      SM.flags.removalInitReceived = true;
      break;
    }

    case MGMT_REMOVE_PARAMS: {
      RemoveParamsPayload p; extractRemoveParamsPayload(pkt, p);
      Serial.printf("[REMOVE] REMOVE_PARAMS received — new neighbor: %d, from target: %d\n",
                    p.newNeighbor, pkt.source);
      SM.flags.removalNewNeighbor    = p.newNeighbor;
      // Only overwrite removalTargetId if not already set by initiateNodeRemoval()
      if (SM.flags.removalTargetId == NODE_NEIGHBOR_NONE)
        SM.flags.removalTargetId = pkt.source;
      // Store original params for possible revert
      SM.flags.removalOrigUpstream   = Node.getUpstream();
      SM.flags.removalOrigDownstream = Node.getDownstream();

      // Master acts as upstream when its immediate downstream is being removed
      if (Node.isMaster() && pkt.source == Node.getDownstream()) {
        Serial.println("[MASTER] REMOVE_PARAMS from immediate downstream — acting as upstream");
        SM.flags.removalInitReceived = true;  // triggers REMOVE_AS_UPSTREAM transition
      } else {
        SM.flags.removalParamsReceived = true;  // regular node handles via EXPLOITATION
      }
      break;
    }

    case MGMT_REMOVE_TEST: {
      Serial.println("[REMOVE] REMOVE_TEST received");
      SM.flags.removalTestReceived = true;
      break;
    }

    case MGMT_REMOVE_TEST_ACK: {
      Serial.println("[REMOVE] REMOVE_TEST_ACK received");
      SM.flags.removalTestAckReceived = true;
      break;
    }

    case MGMT_REMOVE_POWER_OFF: {
      Serial.println("[REMOVE] REMOVE_POWER_OFF received");
      SM.flags.removalPowerOffReceived = true;
      break;
    }

    case MGMT_REMOVE_NOT_POSSIBLE: {
      RemoveNotPossiblePayload p; extractRemoveNotPossiblePayload(pkt, p);
      Serial.println("[REMOVE] REMOVE_NOT_POSSIBLE received");
      SM.flags.removalNotPossibleReceived = true;
      SM.flags.removalOrigUpstream        = p.originalUpstream;
      SM.flags.removalOrigDownstream      = p.originalDownstream;
      break;
    }

    case MGMT_REMOVE_SUCCESS: {
      Serial.println("[REMOVE] REMOVE_SUCCESS received");
      SM.flags.removalSuccessReceived = true;

      if (Node.isMaster()) {
        uint8_t targetId   = SM.flags.removalTargetId;
        uint8_t origUp     = SM.flags.removalOrigUpstream;
        uint8_t origDown   = SM.flags.removalOrigDownstream;

        Serial.printf("[MASTER] Removing node %d from topology (was up=%d, down=%d)\n",
                      targetId, origUp, origDown);

        // Remove the target node
        Topology.removeNode(targetId);

        // Update upstream neighbor's entry — its new downstream = target's old downstream
        if (origUp != NODE_ID_MASTER && origUp != TOPOLOGY_NODE_NONE) {
          NodeEntry upEntry;
          if (Topology.getNode(origUp, upEntry)) {
            Topology.addOrUpdateNode(upEntry.nodeId, upEntry.upstreamId, origDown);
            Serial.printf("[MASTER] Updated node %d downstream: %d -> %d\n",
                          upEntry.nodeId, targetId, origDown);
          }
        }

        // Update downstream neighbor's entry — its new upstream = target's old upstream
        if (origDown != TOPOLOGY_NODE_NONE) {
          NodeEntry downEntry;
          if (Topology.getNode(origDown, downEntry)) {
            Topology.addOrUpdateNode(downEntry.nodeId, origUp, downEntry.downstreamId);
            Serial.printf("[MASTER] Updated node %d upstream: %d -> %d\n",
                          downEntry.nodeId, targetId, origUp);
          }
        }

        // If target was master's immediate downstream, update master's downstream
        if (targetId == Node.getDownstream()) {
          Node.setDownstream(origDown);
          Serial.printf("[MASTER] Updated own downstream: %d -> %d\n", targetId, origDown);
        }

        Topology.printTopology();

        // Schedule TDMA slot re-propagation 10s from now
        tdmaPropagationPending = true;
        tdmaPropagationAt      = millis() + 10000UL;
        Serial.println("[TDMA] Propagation scheduled in 10s");
      }

      Display.clearAll();
      Display.setRow(1, "REMOVAL SUCCESS");
      Display.setRow(2, "Node " + String(SM.flags.removalTargetId) + " removed");
      Display.setFooter(String(FW_VERSION) + " | DONE");
      Display.update();
      break;
    }

    // ── Time sync ────────────────────────────────────────────────────────────
    case MGMT_TIMESYNC_REQUEST: {
      // Record t2 immediately — time of receiving request
      uint64_t t2 = getCurrentTimeMicros();
      TimeSyncRequestPayload p; extractTimeSyncRequestPayload(pkt, p);
      Serial.printf("[TSYNC] REQUEST received from %d, t1=%llu, t2=%llu\n",
                    pkt.source, p.t1, t2);
      SM.flags.timeSyncT2             = t2;
      SM.flags.timeSyncResponseTarget = pkt.source;
      SM.flags.timeSyncRequestReceived = true;
      break;
    }

    case MGMT_TIMESYNC_RESPONSE: {
      // Record t4 immediately — time of receiving response
      uint64_t t4 = getCurrentTimeMicros();
      TimeSyncResponsePayload p; extractTimeSyncResponsePayload(pkt, p);
      Serial.printf("[TSYNC] RESPONSE received: t2=%llu t3=%llu t4=%llu\n",
                    p.t2, p.t3, t4);
      SM.flags.timeSyncT2             = p.t2;
      SM.flags.timeSyncT3             = p.t3;
      SM.flags.timeSyncT4             = t4;
      SM.flags.timeSyncResponseReceived = true;
      break;
    }

    case MGMT_TIMESYNC_START: {
      Serial.println("[TSYNC] TIMESYNC_START received — transitioning to TIME_SYNC_CLIENT");
      SM.flags.timeSyncStartReceived = true;
      break;
    }

    case MGMT_TDMA_SLOT: {
      TdmaSlotPayload p; extractTdmaSlotPayload(pkt, p);
      Serial.printf("[TDMA] Received slot %d\n", p.slot);
      Node.setTdmaSlot(p.slot);

      // Forward next slot to downstream
      uint8_t downstream = Node.getDownstream();
      if (downstream != NODE_NEIGHBOR_NONE) {
        uint8_t nextSlot = (p.slot % 3) + 1;  // 1→2→3→1
        sendTdmaSlot(downstream, downstream, nextSlot);
      }

      Display.clearAll();
      Display.setRow(1, "TDMA SLOT");
      Display.setRow(2, "Slot: " + String(p.slot));
      Display.setRow(3, "Saved to flash");
      Display.update();
      break;
    }

    case MGMT_TIMESYNC_DONE: {
      Serial.printf("[TSYNC] TIMESYNC_DONE received from %d\n", pkt.source);
      SM.flags.timeSyncDoneReceived  = true;
      SM.flags.timeSyncDoneSourceId  = pkt.source;
      Display.clearAll();
      Display.setRow(1, "TSYNC DONE");
      Display.setRow(2, "Network synced");
      Display.setFooter(String(FW_VERSION) + " | EXPLOITATION");
      Display.update();
      break;
    }

    default:
      Serial.printf("[MGMT] Unknown subtype: 0x%02X\n", mgmt.subtype);
      break;
  }
}

void handleTimeSyncPacket(const LoRaPacket& pkt) {
  Serial.println("[TSYNC] Not yet implemented");
  Display.showTemporaryMessage("RX: TIMESYNC", 2000);
}

// ── Command processing ────────────────────────────────────────────────────────

void processCommand(const String& cmd) {
  String command = cmd;
  command.toLowerCase();

  if (command == "help") { printHelp(); }
  else if (command == "config")    { Node.printConfig(); }
  else if (command == "topology")  {
    if (Node.isMaster()) Topology.printTopology();
    else Serial.println("[Error] Only master has topology");
  }
  else if (command.startsWith("setid ")) {
    int id = command.substring(6).toInt();
    if (id >= 0 && id <= 254) Node.setNodeId((uint8_t)id);
    else Serial.println("[Error] ID must be 0-254");
  }
  else if (command.startsWith("setup ")) {
    int id = command.substring(6).toInt();
    if (id >= 0 && id <= 254) Node.setUpstream((uint8_t)id);
    else Serial.println("[Error] ID must be 0-254");
  }
  else if (command.startsWith("setdown ")) {
    int id = command.substring(8).toInt();
    if (id >= 0 && id <= 254) Node.setDownstream((uint8_t)id);
    else Serial.println("[Error] ID must be 0-254");
  }
  else if (command.startsWith("setenrolled ")) {
    int val = command.substring(12).toInt();
    Node.setEnrolled(val != 0);
  }
  else if (command.startsWith("removenode ")) {
    if (!Node.isMaster()) { Serial.println("[Error] Only master can initiate removal"); return; }
    int id = command.substring(11).toInt();
    if (id < 1 || id > 254) { Serial.println("[Error] Target ID must be 1-254"); return; }
    initiateNodeRemoval((uint8_t)id);
  }
  else if (command.startsWith("sdata ")) {
    String args = cmd.substring(6);
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx < 0) { Serial.println("[Error] Usage: sdata <dst> <km>"); return; }
    int dst  = args.substring(0, spaceIdx).toInt();
    int dist = args.substring(spaceIdx + 1).toInt();
    if (dst < 0 || dst > 254)  { Serial.println("[Error] Destination must be 0-254"); return; }
    if (dist < 0 || dist > 40) { Serial.println("[Error] Distance must be 0-40 km");  return; }
    DataPayload data;
    data.distance  = (uint32_t)dist;
    data.timestamp = (uint32_t)(getCurrentTimeMicros() / 1000000ULL);
    memset(data.padding, 0x00, sizeof(data.padding));
    sendDataPacket((uint8_t)dst, data);
  }
  else if (command.startsWith("spacket ")) {
    String args = cmd.substring(8);
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx < 0) { Serial.println("[Error] Usage: spacket <dst> <msg>"); return; }
    int dst    = args.substring(0, spaceIdx).toInt();
    String msg = args.substring(spaceIdx + 1);
    if (dst < 0 || dst > 254) { Serial.println("[Error] Destination must be 0-254"); return; }
    ManagementPayload mgmt;
    mgmt.subtype = 0x00;
    memset(mgmt.data, 0x00, sizeof(mgmt.data));
    size_t copyLen = msg.length() < sizeof(mgmt.data) ? msg.length() : sizeof(mgmt.data);
    memcpy(mgmt.data, msg.c_str(), copyLen);
    sendManagementPacket((uint8_t)dst, mgmt);
  }
  else if (command.startsWith("setssid ")) {
    if (!Node.isMaster()) { Serial.println("[Error] Only master uses WiFi"); return; }
    String s = cmd.substring(8);
    if (s.length() == 0) { Serial.println("[Error] Usage: setssid <ssid>"); return; }
    WifiNtpManager.setSSID(s);
  }
  else if (command.startsWith("setwifipass ")) {
    if (!Node.isMaster()) { Serial.println("[Error] Only master uses WiFi"); return; }
    String p = cmd.substring(12);
    if (p.length() == 0) { Serial.println("[Error] Usage: setwifipass <password>"); return; }
    WifiNtpManager.setPassword(p);
  }
  else if (command == "showwifi") {
    if (!Node.isMaster()) { Serial.println("[Error] Only master uses WiFi"); return; }
    WifiNtpManager.printConfig();
  }
  else if (command.startsWith("settz ")) {
    if (!Node.isMaster()) { Serial.println("[Error] Only master uses WiFi"); return; }
    String tz = cmd.substring(6);
    WifiNtpManager.setTimezone(tz);
  }
  else if (command == "syncclock") {
    // Trigger enrollment-type sync — this node syncs with upstream
    if (Node.isMaster()) { Serial.println("[Error] Master has no upstream to sync with"); return; }
    if (Node.getUpstream() == NODE_NEIGHBOR_NONE) { Serial.println("[Error] No upstream set"); return; }
    SM.flags.timeSyncIsPeriodic  = false;
    SM.flags.timeSyncRequestSent = false;
    SM.transitionToPublic(NodeState::TIME_SYNC_CLIENT);
    Serial.println("[TSYNC] Starting enrollment-type sync with upstream");
  }
  else if (command == "sendslots") {
    if (!Node.isMaster()) { Serial.println("[Error] Only master can send slots"); return; }
    if (Node.getDownstream() == NODE_NEIGHBOR_NONE) { Serial.println("[Error] No downstream neighbor"); return; }
    propagateTdmaFromMaster();
    Serial.println("[TDMA] Slot propagation triggered manually");
  }
  else if (command == "syncnetwork") {
    if (!Node.isMaster()) { Serial.println("[Error] Only master can trigger network sync"); return; }
    if (Node.getDownstream() == NODE_NEIGHBOR_NONE) { Serial.println("[Error] No downstream neighbor"); return; }
    if (SM.networkSyncInProgress || SM.networkRemovalInProgress) {
      Serial.println("[TSYNC] Network action already in progress — ignored");
      Display.showTemporaryMessage("Action in progress!", 2000);
      return;
    }
    SM.startNetworkSync();
    Serial.println("[TSYNC] Network sync started");
  }
  else if (command == "factoryreset") {
    Serial.println("[RESET] Clearing all flash data...");
    Preferences p;
    p.begin("nodeconfig", false); p.clear(); p.end();
    p.begin("topology",   false); p.clear(); p.end();
    p.begin("ntp",        false); p.clear(); p.end();
    Serial.println("[RESET] Done — rebooting");
    Display.showTemporaryMessage("Factory reset...", 1500);
    delay(500);
    ESP.restart();
  }
  else { Serial.println("Unknown command. Type 'help'"); }
}

void printHelp() {
  Serial.println("\n=== Available Commands ===");

  Serial.println("\n-- Node config --");
  Serial.println("  setid <id>           - Set node ID (0=master)");
  Serial.println("  setup <id>           - Set upstream neighbor");
  Serial.println("  setdown <id>         - Set downstream neighbor");
  Serial.println("  setenrolled <0|1>    - Set enrollment flag");
  Serial.println("  config               - Show node config");

  Serial.println("\n-- Network --");
  Serial.println("  topology             - Show network topology (master only)");
  Serial.println("  removenode <id>      - Initiate node removal (master only)");

  Serial.println("\n-- Data --");
  Serial.println("  sdata <dst> <km>     - Send DATA packet");
  Serial.println("  spacket <dst> <msg>  - Send MANAGEMENT packet");

  Serial.println("\n-- Time sync --");
  Serial.println("  syncclock            - Sync this node with upstream");
  Serial.println("  syncnetwork          - Network-wide periodic sync (master only)");
  Serial.println("  sendslots            - Propagate TDMA slots through network (master only)");

  Serial.println("\n-- WiFi / NTP (master only) --");
  Serial.println("  setssid <ssid>       - Set WiFi SSID");
  Serial.println("  setwifipass <pass>   - Set WiFi password");
  Serial.println("  showwifi             - Show WiFi config");
  Serial.println("  settz <tz>           - Set timezone POSIX string");

  Serial.println("\n-- System --");
  Serial.println("  factoryreset         - Clear all flash data and reboot");
  Serial.println("  help                 - Show this help");
  Serial.println("==========================\n");
}