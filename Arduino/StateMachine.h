#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include "NodeConfig.h"

// ── States ────────────────────────────────────────────────────────────────────
enum class NodeState : uint8_t {
  INITIAL,
  EXPLOITATION,
  TIME_SYNC_SERVER,
  ENROLLMENT,
  TIME_SYNC_CLIENT,
  NTP_SYNC,
  REMOVE_SELF,          // This node is being removed
  REMOVE_AS_UPSTREAM,   // I am the upstream neighbor of the node being removed
  REMOVE_AS_DOWNSTREAM, // I am the downstream neighbor of the node being removed
};

// ── Transition flags ──────────────────────────────────────────────────────────
struct StateMachineFlags {
  // INITIAL
  bool enrollmentDataFound  = false;
  bool enrollmentRequested  = false;

  // ENROLLMENT
  bool upstreamAcked        = false;
  bool downstreamAcked      = false;

  // TIME_SYNC
  bool timeSyncComplete     = false;
  bool timeSyncRequested    = false;
  bool downstreamSyncStarted = false;

  // NTP
  bool ntpSyncComplete      = false;
  bool ntpSyncRequested     = false;

  // TIME SYNC
  bool timeSyncIsPeriodic         = false;  // true=periodic, false=enrollment
  bool timeSyncRequestReceived    = false;  // server received request from client
  bool timeSyncResponseReceived   = false;  // client received response from server
  bool timeSyncStartReceived      = false;  // received TIMESYNC_START from upstream
  bool timeSyncDoneReceived       = false;  // master received TIMESYNC_DONE
  uint8_t timeSyncDoneSourceId    = NODE_NEIGHBOR_NONE; // source of TIMESYNC_DONE
  uint64_t timeSyncT1             = 0;      // client: send time
  uint64_t timeSyncT2             = 0;      // server recv time (received in response)
  uint64_t timeSyncT3             = 0;      // server send time (received in response)
  uint64_t timeSyncT4             = 0;      // client: receive time
  bool timeSyncRequestSent        = false;  // client already sent request this cycle
  uint8_t timeSyncResponseTarget  = NODE_NEIGHBOR_NONE; // who sent us the request (server)

  // REMOVAL — set by incoming packet handlers
  bool removalInitReceived      = false;  // REMOVE_INIT received (target)
  bool removalInitSent          = false;  // master already sent REMOVE_INIT for queued removal
  bool removalParamsReceived    = false;  // REMOVE_PARAMS received (neighbors)
  bool removalTestReceived      = false;  // REMOVE_TEST received (downstream)
  bool removalTestAckReceived   = false;  // REMOVE_TEST_ACK received (upstream)
  bool removalPowerOffReceived  = false;  // REMOVE_POWER_OFF received (target)
  bool removalNotPossibleReceived = false; // REMOVE_NOT_POSSIBLE received
  bool removalSuccessReceived   = false;  // REMOVE_SUCCESS received (master)

  // Removal context — populated when REMOVE_PARAMS is received
  uint8_t removalNewNeighbor    = NODE_NEIGHBOR_NONE; // new upstream or downstream
  uint8_t removalTargetId       = NODE_NEIGHBOR_NONE; // ID of node being removed
  // Original params stored for revert
  uint8_t removalOrigUpstream   = NODE_NEIGHBOR_NONE;
  uint8_t removalOrigDownstream = NODE_NEIGHBOR_NONE;
};

class StateMachine {
public:
  StateMachine();

  void begin(const char* version);
  void update();

  NodeState   getState()     const;
  const char* getStateName() const;

  StateMachineFlags flags;

  // Network action flags (master only)
  bool networkSyncInProgress    = false;  // network-wide time sync running
  bool networkRemovalInProgress = false;  // node removal running

  // Public wrappers for external triggering
  void transitionToPublic(NodeState next) { transitionTo(next); }
  void sendTimeSyncStartPublic(uint8_t neighbor) { sendTimeSyncStart(neighbor); }
  void startNetworkSync();  // called by main when syncnetwork is triggered

private:
  NodeState currentState;
  const char* fwVersion;

  // ── Enrollment ────────────────────────────────────────────────────────────
  enum class EnrollStep : uint8_t {
    SEND_UPSTREAM,
    SEND_DOWNSTREAM,
    NOTIFY_MASTER,
    WAIT_BEFORE_SYNC,
    FAILED,
  };
  EnrollStep    enrollStep;
  uint8_t       enrollRetries;
  unsigned long enrollStepStart;

  static const uint8_t  ENROLL_MAX_RETRIES = 2;
  static const uint32_t ENROLL_TIMEOUT_MS  = 15000;

  void runEnrollment();
  void sendNodeAdd(uint8_t neighbor, uint8_t role);
  void sendNodeAdded();

  // ── Removal ───────────────────────────────────────────────────────────────
  static const uint32_t REMOVAL_TIMEOUT_MS = 60000;
  unsigned long removalTimerStart   = 0;
  bool          removalTimerStarted = false;

  void sendTimeSyncRequest();
  void sendTimeSyncResponse();
  void sendTimeSyncStart(uint8_t neighbor);
  void sendTimeSyncDone();
  bool          timeSyncNetworkStartSent = false;  // TIMESYNC_START sent, waiting for TIMESYNC_DONE
  unsigned long networkSyncStartedAt     = 0;        // millis() when TIMESYNC_START was sent
  unsigned long networkRemovalStartedAt  = 0;        // millis() when REMOVE_INIT was sent
  unsigned long timeSyncClientSentAt     = 0;        // millis() when TIMESYNC_REQUEST was sent
  bool          timeSyncClientRetried    = false;    // true after first retry
  void sendRemoveParams(uint8_t neighbor, uint8_t newNeighborId);
  void sendRemoveSimple(uint8_t subtype, uint8_t destination, uint8_t nextHop);
  void sendRemoveNotPossible(uint8_t destination, uint8_t nextHop,
                             uint8_t origUpstream, uint8_t origDownstream);

  // ── Master queue ──────────────────────────────────────────────────────────
  uint8_t   queueHead;
  uint8_t   queueTail;
  uint8_t   queueCount;

  // ── State handlers ────────────────────────────────────────────────────────
  void handleInitial();
  void handleEnrollment();
  void handleTimeSyncClient();
  void handleTimeSyncServer();
  void handleExploitation();
  void handleNtpSync();
  void handleRemoveSelf();
  void handleRemoveAsUpstream();
  void handleRemoveAsDownstream();

  void transitionTo(NodeState next);
  static const char* stateName(NodeState s);
  void updateDisplay();
};

extern StateMachine SM;

#endif