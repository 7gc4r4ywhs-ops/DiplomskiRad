#include "StateMachine.h"
#include "NodeConfig.h"
#include "DisplayManager.h"
#include "LoRaPacket.h"
#include "LoRaByteManager.h"
#include "WifiNtp.h"
#include <Arduino.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <sys/time.h>

extern int64_t clockOffset;
extern void sendRemoveInit(uint8_t targetId);
#include "NetworkTopology.h"
extern NetworkTopology Topology;

extern uint8_t txBuf[];

StateMachine SM;

// ── Constructor ───────────────────────────────────────────────────────────────

StateMachine::StateMachine() {
  currentState        = NodeState::INITIAL;
  fwVersion           = "unknown";
  enrollStep          = EnrollStep::SEND_UPSTREAM;
  enrollRetries       = 0;
  enrollStepStart     = 0;
  removalTimerStart   = 0;
  removalTimerStarted = false;
  timeSyncClientSentAt  = 0;
  timeSyncClientRetried = false;
  networkSyncStartedAt  = 0;
  networkRemovalStartedAt = 0;
}

// ── begin() ───────────────────────────────────────────────────────────────────

void StateMachine::begin(const char* version) {
  fwVersion    = version;
  currentState = NodeState::INITIAL;
  Serial.println("[SM] Starting in INITIAL state");
  updateDisplay();
}

// ── update() ─────────────────────────────────────────────────────────────────

void StateMachine::update() {
  switch (currentState) {
    case NodeState::INITIAL:               handleInitial();            break;
    case NodeState::ENROLLMENT:            handleEnrollment();         break;
    case NodeState::TIME_SYNC_CLIENT:      handleTimeSyncClient();     break;
    case NodeState::TIME_SYNC_SERVER:      handleTimeSyncServer();     break;
    case NodeState::EXPLOITATION:          handleExploitation();       break;
    case NodeState::NTP_SYNC:              handleNtpSync();            break;
    case NodeState::REMOVE_SELF:           handleRemoveSelf();         break;
    case NodeState::REMOVE_AS_UPSTREAM:    handleRemoveAsUpstream();   break;
    case NodeState::REMOVE_AS_DOWNSTREAM:  handleRemoveAsDownstream(); break;
  }
}

// ── INITIAL ───────────────────────────────────────────────────────────────────

void StateMachine::handleInitial() {
  if (Node.isMaster()) {
    networkSyncInProgress = true;
    transitionTo(NodeState::NTP_SYNC);
    return;
  }

  if (Node.getNodeId() == NODE_ID_UNSET) {
    static bool displayed = false;
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 5000) {
      Serial.println("[SM] Node ID not set — use: setid <id>");
      lastLog = millis();
    }
    if (!displayed) {
      Display.clearAll();
      Display.setRow(1, "NOT CONFIGURED");
      Display.setRow(2, "Set node ID:");
      Display.setRow(3, "setid <id>");
      Display.setFooter("Waiting...");
      Display.update();
      displayed = true;
    }
    return;
  }

  if (Node.getUpstream() == NODE_NEIGHBOR_NONE) {
    static bool displayed = false;
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 5000) {
      Serial.println("[SM] Upstream not set — use: setup <id>");
      lastLog = millis();
    }
    if (!displayed) {
      Display.clearAll();
      Display.setRow(1, "NOT CONFIGURED");
      Display.setRow(2, "Set upstream:");
      Display.setRow(3, "setup <id>");
      Display.setFooter("Waiting...");
      Display.update();
      displayed = true;
    }
    return;
  }

  if (flags.enrollmentDataFound) {
    // Enrolled node syncs with upstream on every boot before entering exploitation
    flags.timeSyncIsPeriodic  = false;
    flags.timeSyncRequestSent = false;
    transitionTo(NodeState::TIME_SYNC_CLIENT);
    return;
  }

  // Configured but not enrolled — wait for button press
  if (!flags.enrollmentRequested) {
    static bool displayed = false;
    static unsigned long lastLog = 0;
    if (!displayed) {
      Display.clearAll();
      Display.setRow(1, "Node " + String(Node.getNodeId()));
      Display.setRow(2, "UP: " + String(Node.getUpstream()));
      Display.setRow(3, "Press button to");
      Display.setRow(4, "start enrollment");
      Display.setFooter("Ready to enroll");
      Display.update();
      displayed = true;
    }
    if (millis() - lastLog > 5000) {
      Serial.println("[SM] Press PRG button to start enrollment");
      lastLog = millis();
    }
    return;
  }

  flags.enrollmentRequested = false;
  enrollStep      = EnrollStep::SEND_UPSTREAM;
  enrollRetries   = 0;
  enrollStepStart = 0;
  flags.upstreamAcked   = false;
  flags.downstreamAcked = false;
  transitionTo(NodeState::ENROLLMENT);
}

// ── ENROLLMENT ────────────────────────────────────────────────────────────────

void StateMachine::handleEnrollment() { runEnrollment(); }

void StateMachine::runEnrollment() {
  uint8_t upstream   = Node.getUpstream();
  uint8_t downstream = Node.getDownstream();

  switch (enrollStep) {
    case EnrollStep::SEND_UPSTREAM: {
      if (enrollStepStart == 0 || (millis() - enrollStepStart >= ENROLL_TIMEOUT_MS)) {
        if (enrollStepStart != 0) {
          if (enrollRetries >= ENROLL_MAX_RETRIES) {
            Serial.println("[ENROLL] Upstream ACK failed — enrollment failed");
            enrollStep = EnrollStep::FAILED;
            Display.clearAll();
            Display.setRow(1, "ENROLLMENT");
            Display.setRow(2, "FAILED");
            Display.setRow(3, "Upstream no ACK");
            Display.setRow(4, "Reboot to retry");
            Display.setFooter("Enrollment failed");
            Display.update();
            return;
          }
          enrollRetries++;
          Serial.printf("[ENROLL] Upstream timeout, retry %d/%d\n", enrollRetries, ENROLL_MAX_RETRIES);
        }
        sendNodeAdd(upstream, ROLE_UPSTREAM);
        enrollStepStart = millis();
        Display.clearAll();
        Display.setRow(1, "ENROLLMENT");
        Display.setRow(2, "-> UP " + String(upstream));
        Display.setRow(3, "Waiting ACK...");
        Display.setRow(4, "Retry: " + String(enrollRetries) + "/" + String(ENROLL_MAX_RETRIES));
        Display.update();
      }
      if (flags.upstreamAcked) {
        flags.upstreamAcked = false;
        Serial.println("[ENROLL] Upstream ACKed");
        enrollRetries   = 0;
        enrollStepStart = 0;
        enrollStep = (downstream == NODE_NEIGHBOR_NONE)
                     ? EnrollStep::NOTIFY_MASTER
                     : EnrollStep::SEND_DOWNSTREAM;
      }
      break;
    }

    case EnrollStep::SEND_DOWNSTREAM: {
      if (enrollStepStart == 0 || (millis() - enrollStepStart >= ENROLL_TIMEOUT_MS)) {
        if (enrollStepStart != 0) {
          if (enrollRetries >= ENROLL_MAX_RETRIES) {
            Serial.println("[ENROLL] Downstream ACK failed — enrollment failed");
            enrollStep = EnrollStep::FAILED;
            Display.clearAll();
            Display.setRow(1, "ENROLLMENT");
            Display.setRow(2, "FAILED");
            Display.setRow(3, "Downstream no ACK");
            Display.setRow(4, "Reboot to retry");
            Display.setFooter("Enrollment failed");
            Display.update();
            return;
          }
          enrollRetries++;
          Serial.printf("[ENROLL] Downstream timeout, retry %d/%d\n", enrollRetries, ENROLL_MAX_RETRIES);
        }
        sendNodeAdd(downstream, ROLE_DOWNSTREAM);
        enrollStepStart = millis();
        Display.clearAll();
        Display.setRow(1, "ENROLLMENT");
        Display.setRow(2, "-> DOWN " + String(downstream));
        Display.setRow(3, "Waiting ACK...");
        Display.setRow(4, "Retry: " + String(enrollRetries) + "/" + String(ENROLL_MAX_RETRIES));
        Display.update();
      }
      if (flags.downstreamAcked) {
        flags.downstreamAcked = false;
        Serial.println("[ENROLL] Downstream ACKed");
        enrollRetries   = 0;
        enrollStepStart = 0;
        enrollStep      = EnrollStep::NOTIFY_MASTER;
      }
      break;
    }

    case EnrollStep::NOTIFY_MASTER: {
      sendNodeAdded();
      Node.setEnrolled(true);
      Serial.println("[ENROLL] NODE_ADDED sent — draining TX queue");
      Display.clearAll();
      Display.setRow(1, "ENROLLMENT");
      Display.setRow(2, "Complete!");
      Display.setRow(3, "Finishing up...");
      Display.update();

      // Wait for NODE_ADDED to actually transmit (drain TX queue)
      unsigned long drainStart = millis();
      while (loraByte.getTxQueueCount() > 0 && millis() - drainStart < 8000UL) {
        loraByte.update();
        delay(10);
      }
      Serial.println("[ENROLL] NODE_ADDED transmitted");

      // Non-blocking wait before time sync — lets upstream forward NODE_ADDED
      // without colliding with our TIMESYNC_REQUEST
      enrollStep      = EnrollStep::WAIT_BEFORE_SYNC;
      enrollStepStart = millis();
      Display.setRow(3, "Sync in 5s...");
      Display.update();
      break;
    }

    case EnrollStep::WAIT_BEFORE_SYNC: {
      // Give upstream time to forward NODE_ADDED through the chain
      if (millis() - enrollStepStart >= 5000UL) {
        flags.timeSyncIsPeriodic  = false;
        flags.timeSyncRequestSent = false;
        transitionTo(NodeState::TIME_SYNC_CLIENT);
      }
      break;
    }

    case EnrollStep::FAILED:
      return;
  }
}

void StateMachine::sendNodeAdd(uint8_t neighbor, uint8_t role) {
  uint8_t myId = Node.getNodeId();
  uint8_t recipientRole = (role == ROLE_UPSTREAM) ? ROLE_DOWNSTREAM : ROLE_UPSTREAM;

  NodeAddPayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype = MGMT_NODE_ADD;
  p.nodeId  = myId;
  p.role    = recipientRole;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(NodeAddPayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, neighbor, myId, neighbor, mgmt, txBuf);
  Serial.printf("[ENROLL] Sending NODE_ADD to %d (I am their %s)\n",
                neighbor, recipientRole == ROLE_UPSTREAM ? "UPSTREAM" : "DOWNSTREAM");
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

void StateMachine::sendNodeAdded() {
  uint8_t myId    = Node.getNodeId();
  uint8_t nextHop = Node.getUpstream();

  NodeAddedPayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype      = MGMT_NODE_ADDED;
  p.nodeId       = myId;
  p.upstreamId   = Node.getUpstream();
  p.downstreamId = Node.getDownstream();

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(NodeAddedPayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, NODE_ID_MASTER, myId, nextHop, mgmt, txBuf);
  Serial.println("[ENROLL] Sending NODE_ADDED to master");
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

// ── TIME_SYNC_CLIENT ──────────────────────────────────────────────────────────

void StateMachine::handleTimeSyncClient() {
  // Step 1: On entry or retry — send TIMESYNC_REQUEST with fresh t1
  if (!flags.timeSyncRequestSent) {
    flags.timeSyncT1          = (uint64_t)((int64_t)esp_timer_get_time() + clockOffset);
    timeSyncClientSentAt      = millis();
    timeSyncClientRetried     = false;
    sendTimeSyncRequest();
    flags.timeSyncRequestSent = true;
    Display.clearAll();
    Display.setRow(1, "TIME SYNC");
    Display.setRow(2, flags.timeSyncIsPeriodic ? "Periodic" : "Enrollment");
    Display.setRow(3, "Request sent...");
    Display.update();
    return;
  }

  // Step 1b: 30s elapsed — retry once with fresh t1
  if (!timeSyncClientRetried && millis() - timeSyncClientSentAt >= 30000UL) {
    Serial.println("[TSYNC] No response — retrying with fresh t1");
    flags.timeSyncT1      = (uint64_t)((int64_t)esp_timer_get_time() + clockOffset);
    timeSyncClientSentAt  = millis();
    timeSyncClientRetried = true;
    sendTimeSyncRequest();
    Display.setRow(3, "Retrying...");
    Display.update();
    return;
  }

  // Step 1c: Another 30s elapsed after retry — give up
  if (timeSyncClientRetried && millis() - timeSyncClientSentAt >= 30000UL) {
    Serial.println("[TSYNC] Retry also timed out — returning to EXPLOITATION");
    Display.showTemporaryMessage("TSYNC failed!", 2000);
    flags.timeSyncRequestSent = false;
    timeSyncClientRetried     = false;
    timeSyncClientSentAt      = 0;
    if (flags.timeSyncIsPeriodic) {
      // Send TIMESYNC_DONE regardless of position — master will handle it
      sendTimeSyncDone();
      Serial.println("[TSYNC] Sent TIMESYNC_DONE to master after timeout");
    }
    transitionTo(NodeState::EXPLOITATION);
    return;
  }

  // Step 2: Response received
  if (flags.timeSyncResponseReceived) {
    flags.timeSyncResponseReceived = false;
    flags.timeSyncRequestSent      = false;
    flags.timeSyncT4 = (uint64_t)((int64_t)esp_timer_get_time() + clockOffset);

    // Compute offset: c = (t2 + t3) / 2 - (t1 + t4) / 2
    int64_t t1 = (int64_t)flags.timeSyncT1;
    int64_t t2 = (int64_t)flags.timeSyncT2;
    int64_t t3 = (int64_t)flags.timeSyncT3;
    int64_t t4 = (int64_t)flags.timeSyncT4;
    int64_t offset = (t2/2 + t3/2) - (t1/2 + t4/2);

    // Apply offset to global clock
    clockOffset += offset;

    // Set system clock so getLocalTime() works on this node
    uint64_t nowUs = (uint64_t)((int64_t)esp_timer_get_time() + clockOffset);
    struct timeval tv;
    tv.tv_sec  = (time_t)(nowUs / 1000000ULL);
    tv.tv_usec = (suseconds_t)(nowUs % 1000000ULL);
    settimeofday(&tv, nullptr);
    Serial.printf("[TSYNC] System time set to %lu\n", (unsigned long)tv.tv_sec);

    // Apply timezone so getLocalTime() returns local time correctly
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    Serial.printf("[TSYNC] t1=%lld t2=%lld t3=%lld t4=%lld\n", t1, t2, t3, t4);
    Serial.printf("[TSYNC] Offset: %lld us, Total offset: %lld us\n", offset, clockOffset);

    Display.clearAll();
    Display.setRow(1, "TIME SYNC DONE");
    Display.setRow(2, "Offset: " + String((int32_t)(offset/1000)) + "ms");
    Display.update();
    delay(500);

    if (!flags.timeSyncIsPeriodic) {
      // Enrollment sync — go to EXPLOITATION
      transitionTo(NodeState::EXPLOITATION);
    } else {
      // Periodic sync — propagate downstream or notify master if last node
      uint8_t downstream = Node.getDownstream();
      if (downstream != NODE_NEIGHBOR_NONE) {
        sendTimeSyncStart(downstream);
        Serial.printf("[TSYNC] Propagating to downstream %d\n", downstream);
      } else {
        // Last node — notify master
        sendTimeSyncDone();
        Serial.println("[TSYNC] Last node — sent TIMESYNC_DONE to master");
      }
      transitionTo(NodeState::EXPLOITATION);
    }
  }
}

// ── TIME_SYNC_SERVER ──────────────────────────────────────────────────────────

void StateMachine::handleTimeSyncServer() {
  // Respond to request immediately when received
  if (flags.timeSyncRequestReceived) {
    flags.timeSyncRequestReceived = false;

    // t2 was recorded in the packet handler when request arrived
    // Record t3 now (just before sending response)
    flags.timeSyncT3 = (uint64_t)((int64_t)esp_timer_get_time() + clockOffset);
    sendTimeSyncResponse();

    Serial.printf("[TSYNC] Responded: t2=%llu t3=%llu\n", flags.timeSyncT2, flags.timeSyncT3);

    Display.clearAll();
    Display.setRow(1, "TIME SYNC");
    Display.setRow(2, "Server");
    Display.setRow(3, "Response sent");
    Display.update();

    transitionTo(NodeState::EXPLOITATION);
  }
}

// ── NTP_SYNC ──────────────────────────────────────────────────────────────────

void StateMachine::handleNtpSync() {
  // Drive the WiFi/NTP state machine — transitions to EXPLOITATION when synced
  if (WifiNtpManager.update()) {
    Serial.println("[SM] NTP sync complete");

    // Set clockOffset so getCurrentTimeMicros() returns real Unix microseconds
    // esp_timer_get_time() is microseconds since boot, we need to add the
    // difference between Unix epoch microseconds and boot time
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    uint64_t unixNowUs  = (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
    uint64_t timerNowUs = (uint64_t)esp_timer_get_time();
    clockOffset = (int64_t)(unixNowUs - timerNowUs);
    Serial.printf("[SM] clockOffset set to %lld us (Unix base)\n", clockOffset);

    transitionTo(NodeState::EXPLOITATION);
  }
}

// ── EXPLOITATION ──────────────────────────────────────────────────────────────

void StateMachine::handleExploitation() {
  if (Node.isMaster()) {
    // Always respond to TIMESYNC_REQUEST
    if (flags.timeSyncRequestReceived) {
      transitionTo(NodeState::TIME_SYNC_SERVER);
      return;
    }

    // Network-wide sync in progress — send TIMESYNC_START once, wait for TIMESYNC_DONE
    if (networkSyncInProgress && !timeSyncNetworkStartSent) {
      uint8_t downstream = Node.getDownstream();
      if (downstream != NODE_NEIGHBOR_NONE) {
        Serial.printf("[SM] Sending TIMESYNC_START to %d\n", downstream);
        sendTimeSyncStart(downstream);
        timeSyncNetworkStartSent  = true;
        networkSyncStartedAt      = millis();
      } else {
        networkSyncInProgress    = false;
        timeSyncNetworkStartSent = false;
      }
      return;
    }

    // Master watchdog — clear networkSyncInProgress if sync takes too long
    // Timeout = n * 70s where n = number of enrolled nodes
    if (networkSyncInProgress && timeSyncNetworkStartSent) {
      uint32_t timeoutMs = (uint32_t)Topology.getNodeCount() * 70000UL;
      if (timeoutMs == 0) timeoutMs = 70000UL; // safety — at least 70s
      if (millis() - networkSyncStartedAt >= timeoutMs) {
        Serial.printf("[SM] Network sync watchdog fired (%d nodes, %lus timeout) — clearing flag\n",
                      Topology.getNodeCount(), timeoutMs / 1000);
        networkSyncInProgress    = false;
        timeSyncNetworkStartSent = false;
        networkSyncStartedAt     = 0;
        Display.showTemporaryMessage("Sync watchdog!", 2000);
      }
    }

    // Node removal — only if no sync in progress
    if (networkRemovalInProgress && !networkSyncInProgress &&
        flags.removalTargetId != NODE_NEIGHBOR_NONE &&
        !flags.removalInitSent) {
      flags.removalInitSent  = true;
      networkRemovalStartedAt = millis();
      Serial.printf("[SM] Sending REMOVE_INIT to node %d\n", flags.removalTargetId);
      sendRemoveInit(flags.removalTargetId);
    }

    // Master removal watchdog — 2 minute timeout
    if (networkRemovalInProgress && flags.removalInitSent) {
      if (millis() - networkRemovalStartedAt >= 120000UL) {
        Serial.println("[SM] Removal watchdog fired — clearing removal flag");
        flags.removalInitSent        = false;
        flags.removalTargetId        = NODE_NEIGHBOR_NONE;
        networkRemovalInProgress     = false;
        networkRemovalStartedAt      = 0;
        Display.showTemporaryMessage("Removal timeout!", 2000);
      }
    }


    if (flags.removalSuccessReceived) {
      flags.removalSuccessReceived = false;
      flags.removalInitSent        = false;
      flags.removalTargetId        = NODE_NEIGHBOR_NONE;
      networkRemovalInProgress     = false;
      networkRemovalStartedAt      = 0;
      Serial.println("[MASTER] Removal successful");
    }

    if (flags.removalNotPossibleReceived) {
      flags.removalNotPossibleReceived = false;
      flags.removalInitSent            = false;
      flags.removalTargetId            = NODE_NEIGHBOR_NONE;
      networkRemovalInProgress         = false;
      networkRemovalStartedAt          = 0;
      Serial.println("[MASTER] Removal not possible");
    }

    if (flags.removalInitReceived) {
      flags.removalInitReceived = false;
      // Master's immediate downstream is being removed — act as upstream
      transitionTo(NodeState::REMOVE_AS_UPSTREAM);
      return;
    }

    // Always respond to TIMESYNC_REQUEST regardless of queue state
    if (flags.timeSyncRequestReceived) {
      transitionTo(NodeState::TIME_SYNC_SERVER);
      return;
    }

    if (flags.timeSyncDoneReceived) {
      flags.timeSyncDoneReceived = false;
      timeSyncNetworkStartSent   = false;
      networkSyncInProgress      = false;
      networkSyncStartedAt       = 0;
      Serial.printf("[SM] TIMESYNC_DONE received from %d\n", flags.timeSyncDoneSourceId);

      // Check if sender has a downstream in topology — if so, it timed out early
      // and sync didn't reach the full chain — restart from that node's downstream
      if (flags.timeSyncDoneSourceId != NODE_NEIGHBOR_NONE) {
        NodeEntry entry;
        if (Topology.getNode(flags.timeSyncDoneSourceId, entry) &&
            entry.downstreamId != TOPOLOGY_NODE_NONE) {
          Serial.printf("[SM] Early TIMESYNC_DONE — %d has downstream %d, restarting sync\n",
                        flags.timeSyncDoneSourceId, entry.downstreamId);
          // Restart sync from this node's downstream — no NTP needed
          networkSyncInProgress    = true;
          timeSyncNetworkStartSent = false;
          // Override: send TIMESYNC_START directly to the skipped downstream
          sendTimeSyncStart(entry.downstreamId);
          timeSyncNetworkStartSent = true;
        } else {
          Serial.println("[SM] Network sync complete");
        }
      }
      flags.timeSyncDoneSourceId = NODE_NEIGHBOR_NONE;
    }
    // Daily NTP re-sync after 24h
    {
      static unsigned long lastNtpSync = 0;
      const unsigned long ONE_DAY_MS = 86400000UL;
      if (lastNtpSync == 0) lastNtpSync = millis();
      if (millis() - lastNtpSync >= ONE_DAY_MS && !networkSyncInProgress && !networkRemovalInProgress) {
        lastNtpSync = millis();
        Serial.println("[SM] 24h elapsed — starting NTP re-sync");
        networkSyncInProgress    = true;
        timeSyncNetworkStartSent = false;
        WifiNtpManager.resetSync();
        transitionTo(NodeState::NTP_SYNC);
      }
    }

  } else {
    // Regular node
    if (flags.removalInitReceived) {
      flags.removalInitReceived = false;
      transitionTo(NodeState::REMOVE_SELF);
      return;
    }
    if (flags.removalParamsReceived) {
      flags.removalParamsReceived = false;
      if (flags.removalTargetId == Node.getDownstream()) {
        transitionTo(NodeState::REMOVE_AS_UPSTREAM);
      } else {
        transitionTo(NodeState::REMOVE_AS_DOWNSTREAM);
      }
      return;
    }
    if (flags.timeSyncStartReceived) {
      flags.timeSyncStartReceived  = false;
      flags.timeSyncIsPeriodic     = true;
      flags.timeSyncRequestSent    = false;
      transitionTo(NodeState::TIME_SYNC_CLIENT);
      return;
    }
    if (flags.timeSyncRequestReceived) {
      transitionTo(NodeState::TIME_SYNC_SERVER);
    }
  }
}

// ── REMOVE_SELF ───────────────────────────────────────────────────────────────

void StateMachine::handleRemoveSelf() {
  uint8_t myId       = Node.getNodeId();
  uint8_t upstream   = Node.getUpstream();
  uint8_t downstream = Node.getDownstream();

  // Step 1: On entry
  if (!removalTimerStarted) {
    flags.removalOrigUpstream   = upstream;
    flags.removalOrigDownstream = downstream;

    // Special case: no downstream — simple removal, no connectivity test needed
    if (downstream == NODE_NEIGHBOR_NONE) {
      Serial.println("[REMOVE_SELF] No downstream — simple removal");

      // Tell upstream its new downstream is NONE
      sendRemoveParams(upstream, NODE_NEIGHBOR_NONE);

      // Notify master directly — no need to wait for upstream test
      sendRemoveSimple(MGMT_REMOVE_SUCCESS, NODE_ID_MASTER, upstream);

      Serial.println("[REMOVE_SELF] Simple removal — waiting for TX then shutting down");
      Display.showTemporaryMessage("Sending...", 2000);

      // Wait for TX queue to drain before powering off
      unsigned long waitStart = millis();
      while (loraByte.getTxQueueCount() > 0 && millis() - waitStart < 10000UL) {
        loraByte.update();
        delay(10);
      }

      Display.showTemporaryMessage("Resetting...", 1500);
      Preferences prefs;
      prefs.begin("nodeconfig", false); prefs.clear(); prefs.end();
      prefs.begin("topology",   false); prefs.clear(); prefs.end();
      Display.showTemporaryMessage("Powering off...", 1500);
      esp_deep_sleep_start();
      return;
    }

    // Normal case: has downstream — send params and wait for upstream test
    sendRemoveParams(downstream, upstream);
    Serial.printf("[REMOVE_SELF] Sent params to downstream %d (new upstream=%d)\n",
                  downstream, upstream);

    sendRemoveParams(upstream, downstream);
    Serial.printf("[REMOVE_SELF] Sent params to upstream %d (new downstream=%d)\n",
                  upstream, downstream);

    removalTimerStart   = millis();
    removalTimerStarted = true;

    Display.clearAll();
    Display.setRow(1, "REMOVE SELF");
    Display.setRow(2, "Params sent");
    Display.setRow(3, "Waiting...");
    Display.update();
    return;
  }

  // Step 2: Wait for power-off or not-possible
  if (flags.removalPowerOffReceived) {
    flags.removalPowerOffReceived = false;
    removalTimerStarted = false;
    Serial.println("[REMOVE_SELF] Power-off received — factory reset and shutting down");
    Display.showTemporaryMessage("Resetting...", 1500);

    // Clear all flash data before powering off
    Preferences prefs;
    prefs.begin("nodeconfig", false); prefs.clear(); prefs.end();
    prefs.begin("topology",   false); prefs.clear(); prefs.end();

    // Wait for any queued TX to complete
    unsigned long waitStart = millis();
    while (loraByte.getTxQueueCount() > 0 && millis() - waitStart < 10000UL) {
      loraByte.update();
      delay(10);
    }

    Display.showTemporaryMessage("Powering off...", 1500);
    esp_deep_sleep_start();
  }

  if (flags.removalNotPossibleReceived) {
    flags.removalNotPossibleReceived = false;
    removalTimerStarted = false;
    Serial.println("[REMOVE_SELF] Removal not possible — notifying master and downstream");

    // Notify master
    sendRemoveNotPossible(NODE_ID_MASTER, upstream,
                          flags.removalOrigUpstream, flags.removalOrigDownstream);

    // Notify downstream to revert
    if (downstream != NODE_NEIGHBOR_NONE) {
      sendRemoveNotPossible(downstream, downstream,
                            flags.removalOrigUpstream, flags.removalOrigDownstream);
    }

    transitionTo(NodeState::EXPLOITATION);
  }

  // Timeout — send removal not possible ourselves
  if (removalTimerStarted && millis() - removalTimerStart >= REMOVAL_TIMEOUT_MS) {
    removalTimerStarted = false;
    Serial.println("[REMOVE_SELF] Timeout — sending removal not possible");

    sendRemoveNotPossible(NODE_ID_MASTER, upstream,
                          flags.removalOrigUpstream, flags.removalOrigDownstream);
    if (downstream != NODE_NEIGHBOR_NONE) {
      sendRemoveNotPossible(downstream, downstream,
                            flags.removalOrigUpstream, flags.removalOrigDownstream);
    }

    transitionTo(NodeState::EXPLOITATION);
  }
}

// ── REMOVE_AS_UPSTREAM ────────────────────────────────────────────────────────

void StateMachine::handleRemoveAsUpstream() {
  uint8_t myId = Node.getNodeId();

  // Step 1: On entry
  if (!removalTimerStarted) {
    uint8_t newDownstream = flags.removalNewNeighbor;

    // If new downstream is NONE — target was last node, just update and done
    if (newDownstream == NODE_NEIGHBOR_NONE) {
      Serial.println("[REMOVE_AS_UP] New downstream is NONE — last node removed, updating config");
      Node.setDownstream(NODE_NEIGHBOR_NONE);
      removalTimerStarted = false;
      Display.clearAll();
      Display.setRow(1, "REMOVE: DONE");
      Display.setRow(2, "Last node removed");
      Display.setRow(3, "DOWN: none");
      Display.update();
      transitionTo(NodeState::EXPLOITATION);
      return;
    }

    // Normal case — send REMOVE_TEST to new downstream
    Serial.printf("[REMOVE_AS_UP] Sending REMOVE_TEST to new downstream %d\n", newDownstream);
    sendRemoveSimple(MGMT_REMOVE_TEST, newDownstream, newDownstream);

    removalTimerStart   = millis();
    removalTimerStarted = true;

    Display.clearAll();
    Display.setRow(1, "REMOVE: UPSTREAM");
    Display.setRow(2, "Test -> " + String(newDownstream));
    Display.setRow(3, "Waiting ACK...");
    Display.update();
    return;
  }

  // Step 2: ACK received — send power-off to target
  if (flags.removalTestAckReceived) {
    flags.removalTestAckReceived = false;
    removalTimerStarted = false;

    uint8_t target  = flags.removalTargetId;
    uint8_t nextHop = Node.getDownstream(); // original downstream = target

    Serial.printf("[REMOVE_AS_UP] ACK received — sending power-off to target %d\n", target);
    sendRemoveSimple(MGMT_REMOVE_POWER_OFF, target, nextHop);

    // Notify master of success
    sendRemoveSimple(MGMT_REMOVE_SUCCESS, NODE_ID_MASTER,
                     Node.isMaster() ? flags.removalNewNeighbor : Node.getUpstream());

    // Update our downstream to the new neighbor
    Node.setDownstream(flags.removalNewNeighbor);

    Display.clearAll();
    Display.setRow(1, "REMOVE: SUCCESS");
    Display.setRow(2, "Power-off sent");
    Display.setRow(3, "New DOWN: " + String(flags.removalNewNeighbor));
    Display.update();

    transitionTo(NodeState::EXPLOITATION);
    return;
  }

  // Step 3: Timeout — revert and send not possible
  if (removalTimerStarted && millis() - removalTimerStart >= REMOVAL_TIMEOUT_MS) {
    removalTimerStarted = false;
    Serial.println("[REMOVE_AS_UP] Timeout — reverting and sending not possible");

    // Revert our downstream to original
    Node.setDownstream(flags.removalTargetId);

    // Send removal not possible to target
    uint8_t target = flags.removalTargetId;
    sendRemoveNotPossible(target, target,
                          flags.removalOrigUpstream, flags.removalOrigDownstream);

    Display.clearAll();
    Display.setRow(1, "REMOVE: FAILED");
    Display.setRow(2, "No ACK from DOWN");
    Display.setRow(3, "Reverted params");
    Display.update();

    transitionTo(NodeState::EXPLOITATION);
  }
}

// ── REMOVE_AS_DOWNSTREAM ─────────────────────────────────────────────────────

void StateMachine::handleRemoveAsDownstream() {
  uint8_t myId = Node.getNodeId();

  // Wait for REMOVE_TEST from new upstream
  if (flags.removalTestReceived) {
    flags.removalTestReceived = false;

    uint8_t newUpstream = flags.removalNewNeighbor;
    Serial.printf("[REMOVE_AS_DOWN] Test received — sending ACK to %d\n", newUpstream);
    sendRemoveSimple(MGMT_REMOVE_TEST_ACK, newUpstream, newUpstream);

    // Update our upstream to the new neighbor
    Node.setUpstream(newUpstream);

    Display.clearAll();
    Display.setRow(1, "REMOVE: DOWNSTREAM");
    Display.setRow(2, "ACK sent");
    Display.setRow(3, "New UP: " + String(newUpstream));
    Display.update();

    transitionTo(NodeState::EXPLOITATION);
    return;
  }

  // Removal not possible — revert
  if (flags.removalNotPossibleReceived) {
    flags.removalNotPossibleReceived = false;
    Serial.println("[REMOVE_AS_DOWN] Not possible — reverting params");

    Node.setUpstream(flags.removalOrigUpstream);

    Display.clearAll();
    Display.setRow(1, "REMOVE: FAILED");
    Display.setRow(2, "Reverted params");
    Display.update();

    transitionTo(NodeState::EXPLOITATION);
  }
}

// ── Removal packet helpers ────────────────────────────────────────────────────

// ── Time sync send helpers ────────────────────────────────────────────────────

void StateMachine::sendTimeSyncRequest() {
  uint8_t myId     = Node.getNodeId();
  uint8_t upstream = Node.getUpstream();

  TimeSyncRequestPayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype = MGMT_TIMESYNC_REQUEST;
  p.t1      = flags.timeSyncT1;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(TimeSyncRequestPayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, upstream, myId, upstream, mgmt, txBuf);
  Serial.printf("[TSYNC] Sending REQUEST to %d, t1=%llu\n", upstream, p.t1);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

void StateMachine::sendTimeSyncResponse() {
  uint8_t myId = Node.getNodeId();
  // timeSyncResponseTarget set when request was received
  uint8_t target = flags.timeSyncResponseTarget;

  TimeSyncResponsePayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype = MGMT_TIMESYNC_RESPONSE;
  p.t2      = flags.timeSyncT2;
  p.t3      = flags.timeSyncT3;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(TimeSyncResponsePayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, target, myId, target, mgmt, txBuf);
  Serial.printf("[TSYNC] Sending RESPONSE to %d, t2=%llu t3=%llu\n", target, p.t2, p.t3);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

void StateMachine::sendTimeSyncStart(uint8_t neighbor) {
  uint8_t myId = Node.getNodeId();

  TimeSyncSimplePayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype = MGMT_TIMESYNC_START;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(TimeSyncSimplePayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, neighbor, myId, neighbor, mgmt, txBuf);
  Serial.printf("[TSYNC] Sending TIMESYNC_START to %d\n", neighbor);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

void StateMachine::sendTimeSyncDone() {
  uint8_t myId    = Node.getNodeId();
  uint8_t nextHop = Node.getUpstream();

  TimeSyncSimplePayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype = MGMT_TIMESYNC_DONE;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(TimeSyncSimplePayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, NODE_ID_MASTER, myId, nextHop, mgmt, txBuf);
  Serial.println("[TSYNC] Sending TIMESYNC_DONE to master");
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

// ── Removal send helpers ──────────────────────────────────────────────────────

void StateMachine::sendRemoveParams(uint8_t neighbor, uint8_t newNeighborId) {
  uint8_t myId = Node.getNodeId();

  RemoveParamsPayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype      = MGMT_REMOVE_PARAMS;
  p.newNeighbor  = newNeighborId;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(RemoveParamsPayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, neighbor, myId, neighbor, mgmt, txBuf);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

void StateMachine::sendRemoveSimple(uint8_t subtype, uint8_t destination, uint8_t nextHop) {
  uint8_t myId = Node.getNodeId();

  RemoveSimplePayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype = subtype;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(RemoveSimplePayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, destination, myId, nextHop, mgmt, txBuf);
  Serial.printf("[REMOVE] Sending subtype 0x%02X to %d\n", subtype, destination);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}

void StateMachine::sendRemoveNotPossible(uint8_t destination, uint8_t nextHop,
                                         uint8_t origUpstream, uint8_t origDownstream) {
  uint8_t myId = Node.getNodeId();

  RemoveNotPossiblePayload p;
  memset(&p, 0x00, sizeof(p));
  p.subtype           = MGMT_REMOVE_NOT_POSSIBLE;
  p.originalUpstream  = origUpstream;
  p.originalDownstream = origDownstream;

  ManagementPayload mgmt;
  memset(&mgmt, 0x00, sizeof(mgmt));
  memcpy(&mgmt, &p, sizeof(RemoveNotPossiblePayload));

  LoRaPacket pkt;
  buildManagementPacket(pkt, myId, destination, myId, nextHop, mgmt, txBuf);
  Serial.printf("[REMOVE] Sending NOT_POSSIBLE to %d\n", destination);
  loraByte.sendPacket(txBuf, PKT_TOTAL_LEN, Node.getNodeId());
}


// ── Network sync trigger (called from main) ──────────────────────────────────

void StateMachine::startNetworkSync() {
  networkSyncInProgress    = true;
  timeSyncNetworkStartSent = false;
  WifiNtpManager.resetSync();
  transitionTo(NodeState::NTP_SYNC);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

NodeState   StateMachine::getState()     const { return currentState; }
const char* StateMachine::getStateName() const { return stateName(currentState); }

const char* StateMachine::stateName(NodeState s) {
  switch (s) {
    case NodeState::INITIAL:               return "INITIAL";
    case NodeState::ENROLLMENT:            return "ENROLLMENT";
    case NodeState::TIME_SYNC_CLIENT:      return "TIME_SYNC_CLIENT";
    case NodeState::TIME_SYNC_SERVER:      return "TIME_SYNC_SERVER";
    case NodeState::EXPLOITATION:          return "EXPLOITATION";
    case NodeState::NTP_SYNC:              return "NTP_SYNC";
    case NodeState::REMOVE_SELF:           return "REMOVE_SELF";
    case NodeState::REMOVE_AS_UPSTREAM:    return "REMOVE_AS_UP";
    case NodeState::REMOVE_AS_DOWNSTREAM:  return "REMOVE_AS_DOWN";
    default:                               return "UNKNOWN";
  }
}

void StateMachine::updateDisplay() {
  Display.clearAll();
  Display.setRow(1, Node.isMaster() ? "MASTER" : "Node " + String(Node.getNodeId()));
  Display.setRow(2, getStateName());
  Display.setFooter(String(fwVersion) + " | " + getStateName());
  Display.update();
}

void StateMachine::transitionTo(NodeState next) {
  Serial.printf("[SM] %s -> %s\n", stateName(currentState), stateName(next));
  // Reset removal timer on any transition
  removalTimerStarted = false;
  removalTimerStart   = 0;
  currentState = next;
  updateDisplay();
}