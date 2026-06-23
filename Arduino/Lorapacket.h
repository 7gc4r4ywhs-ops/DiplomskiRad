#ifndef LORA_PACKET_H
#define LORA_PACKET_H

#include <Arduino.h>

// ── Packet dimensions ─────────────────────────────────────────────────────────
#define PKT_PAYLOAD_LEN   32
#define PKT_HEADER_LEN    6
#define PKT_TRAILER_LEN   2
#define PKT_TOTAL_LEN     (PKT_HEADER_LEN + PKT_PAYLOAD_LEN + PKT_TRAILER_LEN)  // 40 bytes

// ── Header field offsets ──────────────────────────────────────────────────────
#define PKT_OFF_SRC       0
#define PKT_OFF_DST       1
#define PKT_OFF_PREV_HOP  2
#define PKT_OFF_NEXT_HOP  3
#define PKT_OFF_TTL       4
#define PKT_OFF_MSG_TYPE  5

// ── Payload / trailer offsets ─────────────────────────────────────────────────
#define PKT_OFF_PAYLOAD   PKT_HEADER_LEN
#define PKT_OFF_CRC       (PKT_HEADER_LEN + PKT_PAYLOAD_LEN)

// ── Message types ─────────────────────────────────────────────────────────────
#define PKT_TYPE_DATA       0x01
#define PKT_TYPE_MANAGEMENT 0x02
#define PKT_TYPE_TIMESYNC   0x03

// ── Management subtypes ───────────────────────────────────────────────────────
#define MGMT_NODE_ADD          0x01  // Enrolling node announcing itself to a neighbor
#define MGMT_NODE_ADD_ACK      0x02  // Neighbor ACKing the enrollment
#define MGMT_NODE_ADDED        0x03  // Enrolling node notifying master

#define MGMT_REMOVE_INIT       0x04  // Master -> target: start removal process
#define MGMT_REMOVE_PARAMS     0x05  // Target -> neighbor: updated network param
#define MGMT_REMOVE_TEST       0x06  // Upstream -> downstream: connectivity test
#define MGMT_REMOVE_TEST_ACK   0x07  // Downstream -> upstream: test passed
#define MGMT_REMOVE_POWER_OFF  0x08  // Upstream -> target: shut down
#define MGMT_REMOVE_NOT_POSSIBLE 0x09 // Upstream -> target / target -> master+downstream
#define MGMT_REMOVE_SUCCESS    0x0A  // Upstream -> master: removal complete

// ── Time sync subtypes ────────────────────────────────────────────────────────
#define MGMT_TIMESYNC_REQUEST   0x0B  // Client -> server: sync request (carries t1)
#define MGMT_TIMESYNC_RESPONSE  0x0C  // Server -> client: sync response (carries t2, t3)
#define MGMT_TIMESYNC_START     0x0D  // Upstream -> downstream: start sync as client
#define MGMT_TIMESYNC_DONE      0x0E  // Last node -> master: periodic sync complete

// ── TDMA slot assignment ──────────────────────────────────────────────────────
#define MGMT_TDMA_SLOT         0x0F  // Master/upstream -> downstream: assign TDMA slot

// ── Role byte used in NODE_ADD payload ───────────────────────────────────────
#define ROLE_UPSTREAM     0x01  // Sender is the recipient's new upstream neighbor
#define ROLE_DOWNSTREAM   0x02  // Sender is the recipient's new downstream neighbor

// ── Fixed values ──────────────────────────────────────────────────────────────
#define PKT_TTL_INITIAL   5
#define PKT_TTL_EXPIRED   0

// ── Payload structs ───────────────────────────────────────────────────────────

// DATA payload
struct DataPayload {
  uint32_t distance;
  uint32_t timestamp;
  uint8_t  padding[PKT_PAYLOAD_LEN - sizeof(uint32_t) - sizeof(uint32_t)];
};

// Generic MANAGEMENT payload
struct ManagementPayload {
  uint8_t subtype;
  uint8_t data[PKT_PAYLOAD_LEN - 1];
};

// NODE_ADD payload
struct NodeAddPayload {
  uint8_t subtype;    // MGMT_NODE_ADD
  uint8_t nodeId;     // ID of the enrolling node
  uint8_t role;       // ROLE_UPSTREAM or ROLE_DOWNSTREAM
  uint8_t padding[PKT_PAYLOAD_LEN - 3];
};

// NODE_ADD_ACK payload
struct NodeAddAckPayload {
  uint8_t subtype;    // MGMT_NODE_ADD_ACK
  uint8_t neighborId; // ID of the ACKing neighbor
  uint8_t padding[PKT_PAYLOAD_LEN - 2];
};

// NODE_ADDED payload
struct NodeAddedPayload {
  uint8_t subtype;       // MGMT_NODE_ADDED
  uint8_t nodeId;        // ID of the newly enrolled node
  uint8_t upstreamId;    // Its upstream neighbor
  uint8_t downstreamId;  // Its downstream neighbor (NODE_NEIGHBOR_NONE if none)
  uint8_t padding[PKT_PAYLOAD_LEN - 4];
};

// TIMESYNC_REQUEST payload — client -> server
// t1: time of sending request, in microseconds (client clock)
struct TimeSyncRequestPayload {
  uint8_t  subtype;   // MGMT_TIMESYNC_REQUEST
  uint8_t  padding1[3];
  uint64_t t1;        // microseconds, client clock
  uint8_t  padding2[PKT_PAYLOAD_LEN - 1 - 3 - 8];
};

// TIMESYNC_RESPONSE payload — server -> client
// t2: time of receiving request, server clock
// t3: time of sending response, server clock
struct TimeSyncResponsePayload {
  uint8_t  subtype;   // MGMT_TIMESYNC_RESPONSE
  uint8_t  padding1[3];
  uint64_t t2;        // microseconds, server clock
  uint64_t t3;        // microseconds, server clock
  uint8_t  padding2[PKT_PAYLOAD_LEN - 1 - 3 - 8 - 8];
};

// TIMESYNC_START and TIMESYNC_DONE — no extra payload beyond subtype
struct TimeSyncSimplePayload {
  uint8_t subtype;
  uint8_t padding[PKT_PAYLOAD_LEN - 1];
};

// REMOVE_INIT payload (MGMT_REMOVE_INIT)
// Sent by master to the target node
struct RemoveInitPayload {
  uint8_t subtype;    // MGMT_REMOVE_INIT
  uint8_t targetId;   // ID of node to be removed (redundant but explicit)
  uint8_t padding[PKT_PAYLOAD_LEN - 2];
};

// REMOVE_PARAMS payload (MGMT_REMOVE_PARAMS)
// Sent by target to each neighbor — each neighbor only gets what it needs
struct RemoveParamsPayload {
  uint8_t subtype;     // MGMT_REMOVE_PARAMS
  uint8_t newNeighbor; // For upstream: its new downstream. For downstream: its new upstream.
  uint8_t padding[PKT_PAYLOAD_LEN - 2];
};

// REMOVE_NOT_POSSIBLE payload (MGMT_REMOVE_NOT_POSSIBLE)
// Sent by upstream -> target, and target -> master + downstream
// Carries original params so recipients know what to revert to
struct RemoveNotPossiblePayload {
  uint8_t subtype;          // MGMT_REMOVE_NOT_POSSIBLE
  uint8_t originalUpstream;   // Target's original upstream (for downstream to revert)
  uint8_t originalDownstream; // Target's original downstream (for upstream to revert)
  uint8_t padding[PKT_PAYLOAD_LEN - 3];
};

// Subtypes with no extra payload beyond subtype byte:
// MGMT_REMOVE_TEST, MGMT_REMOVE_TEST_ACK, MGMT_REMOVE_POWER_OFF, MGMT_REMOVE_SUCCESS
struct RemoveSimplePayload {
  uint8_t subtype;
  uint8_t padding[PKT_PAYLOAD_LEN - 1];
};

// ── Packet struct ─────────────────────────────────────────────────────────────
struct LoRaPacket {
  uint8_t  source;
  uint8_t  destination;
  uint8_t  prevHop;
  uint8_t  nextHop;
  uint8_t  ttl;
  uint8_t  msgType;
  uint8_t  payload[PKT_PAYLOAD_LEN];
  uint16_t crc;
};

// ── Build functions ───────────────────────────────────────────────────────────
bool buildPacket(LoRaPacket& pkt, uint8_t source, uint8_t destination,
                 uint8_t prevHop, uint8_t nextHop, uint8_t msgType,
                 const uint8_t* payloadBuf, uint8_t* outBuf);

bool buildDataPacket(LoRaPacket& pkt, uint8_t source, uint8_t destination,
                     uint8_t prevHop, uint8_t nextHop,
                     const DataPayload& data, uint8_t* outBuf);

bool buildManagementPacket(LoRaPacket& pkt, uint8_t source, uint8_t destination,
                           uint8_t prevHop, uint8_t nextHop,
                           const ManagementPayload& mgmt, uint8_t* outBuf);

// ── Parse / serialize ─────────────────────────────────────────────────────────
bool parsePacket(const uint8_t* buf, size_t len, LoRaPacket& pkt);
void serializePacket(const LoRaPacket& pkt, uint8_t* buf);

// ── Payload extraction helpers ────────────────────────────────────────────────
void extractDataPayload(const LoRaPacket& pkt, DataPayload& out);
void extractManagementPayload(const LoRaPacket& pkt, ManagementPayload& out);
void extractNodeAddPayload(const LoRaPacket& pkt, NodeAddPayload& out);
void extractNodeAddAckPayload(const LoRaPacket& pkt, NodeAddAckPayload& out);
void extractNodeAddedPayload(const LoRaPacket& pkt, NodeAddedPayload& out);
void extractRemoveInitPayload(const LoRaPacket& pkt, RemoveInitPayload& out);
void extractRemoveParamsPayload(const LoRaPacket& pkt, RemoveParamsPayload& out);
void extractRemoveNotPossiblePayload(const LoRaPacket& pkt, RemoveNotPossiblePayload& out);
void extractRemoveSimplePayload(const LoRaPacket& pkt, RemoveSimplePayload& out);

// TDMA_SLOT payload
struct TdmaSlotPayload {
  uint8_t subtype;    // MGMT_TDMA_SLOT
  uint8_t slot;       // 1, 2, or 3
  uint8_t padding[PKT_PAYLOAD_LEN - 2];
};

void extractTdmaSlotPayload(const LoRaPacket& pkt, TdmaSlotPayload& out);
void extractTimeSyncRequestPayload(const LoRaPacket& pkt, TimeSyncRequestPayload& out);
void extractTimeSyncResponsePayload(const LoRaPacket& pkt, TimeSyncResponsePayload& out);

// ── CRC ───────────────────────────────────────────────────────────────────────
uint16_t computeCRC16(const uint8_t* data, size_t len);

// ── Debug ─────────────────────────────────────────────────────────────────────
void printPacket(const LoRaPacket& pkt);

#endif