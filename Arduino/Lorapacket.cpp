#include "LoRaPacket.h"
#include <Arduino.h>
#include "esp_crc.h"

uint16_t computeCRC16(const uint8_t* data, size_t len) {
  return esp_crc16_le(0xFFFF, data, len);
}

void serializePacket(const LoRaPacket& pkt, uint8_t* buf) {
  buf[PKT_OFF_SRC]      = pkt.source;
  buf[PKT_OFF_DST]      = pkt.destination;
  buf[PKT_OFF_PREV_HOP] = pkt.prevHop;
  buf[PKT_OFF_NEXT_HOP] = pkt.nextHop;
  buf[PKT_OFF_TTL]      = pkt.ttl;
  buf[PKT_OFF_MSG_TYPE] = pkt.msgType;
  memcpy(buf + PKT_OFF_PAYLOAD, pkt.payload, PKT_PAYLOAD_LEN);
  buf[PKT_OFF_CRC]     = pkt.crc & 0xFF;
  buf[PKT_OFF_CRC + 1] = (pkt.crc >> 8) & 0xFF;
}

bool buildPacket(LoRaPacket& pkt, uint8_t source, uint8_t destination,
                 uint8_t prevHop, uint8_t nextHop, uint8_t msgType,
                 const uint8_t* payloadBuf, uint8_t* outBuf) {
  pkt.source      = source;
  pkt.destination = destination;
  pkt.prevHop     = prevHop;
  pkt.nextHop     = nextHop;
  pkt.ttl         = PKT_TTL_INITIAL;
  pkt.msgType     = msgType;
  memcpy(pkt.payload, payloadBuf, PKT_PAYLOAD_LEN);
  pkt.crc = computeCRC16(pkt.payload, PKT_PAYLOAD_LEN);
  serializePacket(pkt, outBuf);
  return true;
}

bool buildDataPacket(LoRaPacket& pkt, uint8_t source, uint8_t destination,
                     uint8_t prevHop, uint8_t nextHop,
                     const DataPayload& data, uint8_t* outBuf) {
  uint8_t buf[PKT_PAYLOAD_LEN];
  memset(buf, 0x00, PKT_PAYLOAD_LEN);
  memcpy(buf, &data, sizeof(DataPayload));
  return buildPacket(pkt, source, destination, prevHop, nextHop,
                     PKT_TYPE_DATA, buf, outBuf);
}

bool buildManagementPacket(LoRaPacket& pkt, uint8_t source, uint8_t destination,
                           uint8_t prevHop, uint8_t nextHop,
                           const ManagementPayload& mgmt, uint8_t* outBuf) {
  uint8_t buf[PKT_PAYLOAD_LEN];
  memset(buf, 0x00, PKT_PAYLOAD_LEN);
  memcpy(buf, &mgmt, sizeof(ManagementPayload));
  return buildPacket(pkt, source, destination, prevHop, nextHop,
                     PKT_TYPE_MANAGEMENT, buf, outBuf);
}

bool parsePacket(const uint8_t* buf, size_t len, LoRaPacket& pkt) {
  if (len != PKT_TOTAL_LEN) {
    Serial.printf("[LoRaPacket] ERROR: Expected %d bytes, got %d\n", PKT_TOTAL_LEN, len);
    return false;
  }
  pkt.source      = buf[PKT_OFF_SRC];
  pkt.destination = buf[PKT_OFF_DST];
  pkt.prevHop     = buf[PKT_OFF_PREV_HOP];
  pkt.nextHop     = buf[PKT_OFF_NEXT_HOP];
  pkt.ttl         = buf[PKT_OFF_TTL];
  pkt.msgType     = buf[PKT_OFF_MSG_TYPE];
  memcpy(pkt.payload, buf + PKT_OFF_PAYLOAD, PKT_PAYLOAD_LEN);
  pkt.crc = (uint16_t)buf[PKT_OFF_CRC] | ((uint16_t)buf[PKT_OFF_CRC + 1] << 8);

  uint16_t computed = computeCRC16(pkt.payload, PKT_PAYLOAD_LEN);
  if (computed != pkt.crc) {
    Serial.printf("[LoRaPacket] CRC FAIL: expected 0x%04X, got 0x%04X\n", computed, pkt.crc);
    return false;
  }
  return true;
}

// ── Payload extraction ────────────────────────────────────────────────────────

void extractDataPayload(const LoRaPacket& pkt, DataPayload& out) {
  memcpy(&out, pkt.payload, sizeof(DataPayload));
}
void extractManagementPayload(const LoRaPacket& pkt, ManagementPayload& out) {
  memcpy(&out, pkt.payload, sizeof(ManagementPayload));
}
void extractNodeAddPayload(const LoRaPacket& pkt, NodeAddPayload& out) {
  memcpy(&out, pkt.payload, sizeof(NodeAddPayload));
}
void extractNodeAddAckPayload(const LoRaPacket& pkt, NodeAddAckPayload& out) {
  memcpy(&out, pkt.payload, sizeof(NodeAddAckPayload));
}
void extractNodeAddedPayload(const LoRaPacket& pkt, NodeAddedPayload& out) {
  memcpy(&out, pkt.payload, sizeof(NodeAddedPayload));
}
void extractRemoveInitPayload(const LoRaPacket& pkt, RemoveInitPayload& out) {
  memcpy(&out, pkt.payload, sizeof(RemoveInitPayload));
}
void extractRemoveParamsPayload(const LoRaPacket& pkt, RemoveParamsPayload& out) {
  memcpy(&out, pkt.payload, sizeof(RemoveParamsPayload));
}
void extractRemoveNotPossiblePayload(const LoRaPacket& pkt, RemoveNotPossiblePayload& out) {
  memcpy(&out, pkt.payload, sizeof(RemoveNotPossiblePayload));
}
void extractRemoveSimplePayload(const LoRaPacket& pkt, RemoveSimplePayload& out) {
  memcpy(&out, pkt.payload, sizeof(RemoveSimplePayload));
}

void extractTdmaSlotPayload(const LoRaPacket& pkt, TdmaSlotPayload& out) {
  memcpy(&out, pkt.payload, sizeof(TdmaSlotPayload));
}

void extractTimeSyncRequestPayload(const LoRaPacket& pkt, TimeSyncRequestPayload& out) {
  memcpy(&out, pkt.payload, sizeof(TimeSyncRequestPayload));
}
void extractTimeSyncResponsePayload(const LoRaPacket& pkt, TimeSyncResponsePayload& out) {
  memcpy(&out, pkt.payload, sizeof(TimeSyncResponsePayload));
}

// ── Debug print ───────────────────────────────────────────────────────────────

void printPacket(const LoRaPacket& pkt) {
  Serial.println("=== LoRa Packet ===");
  Serial.printf("Source:      %d\n", pkt.source);
  Serial.printf("Destination: %d\n", pkt.destination);
  Serial.printf("Prev Hop:    %d\n", pkt.prevHop);
  Serial.printf("Next Hop:    %d\n", pkt.nextHop);
  Serial.printf("TTL:         %d\n", pkt.ttl);

  switch (pkt.msgType) {
    case PKT_TYPE_DATA: {
      Serial.println("Msg Type:    DATA (0x01)");
      DataPayload data; extractDataPayload(pkt, data);
      Serial.printf("Distance:    %d km\n", data.distance);
      Serial.printf("Timestamp:   %u\n",    data.timestamp);
      break;
    }
    case PKT_TYPE_MANAGEMENT: {
      Serial.println("Msg Type:    MANAGEMENT (0x02)");
      ManagementPayload mgmt; extractManagementPayload(pkt, mgmt);
      switch (mgmt.subtype) {
        case MGMT_NODE_ADD: {
          Serial.println("Subtype:     NODE_ADD");
          NodeAddPayload p; extractNodeAddPayload(pkt, p);
          Serial.printf("Node ID:     %d\n", p.nodeId);
          Serial.printf("Role:        %s\n", p.role == ROLE_UPSTREAM ? "UPSTREAM" : "DOWNSTREAM");
          break;
        }
        case MGMT_NODE_ADD_ACK: {
          Serial.println("Subtype:     NODE_ADD_ACK");
          NodeAddAckPayload p; extractNodeAddAckPayload(pkt, p);
          Serial.printf("Neighbor ID: %d\n", p.neighborId);
          break;
        }
        case MGMT_NODE_ADDED: {
          Serial.println("Subtype:     NODE_ADDED");
          NodeAddedPayload p; extractNodeAddedPayload(pkt, p);
          Serial.printf("Node ID:     %d\n", p.nodeId);
          Serial.printf("Upstream:    %d\n", p.upstreamId);
          Serial.printf("Downstream:  %d\n", p.downstreamId);
          break;
        }
        case MGMT_REMOVE_INIT:
          Serial.println("Subtype:     REMOVE_INIT");
          break;
        case MGMT_REMOVE_PARAMS: {
          Serial.println("Subtype:     REMOVE_PARAMS");
          RemoveParamsPayload p; extractRemoveParamsPayload(pkt, p);
          Serial.printf("New Neighbor: %d\n", p.newNeighbor);
          break;
        }
        case MGMT_REMOVE_TEST:
          Serial.println("Subtype:     REMOVE_TEST");
          break;
        case MGMT_REMOVE_TEST_ACK:
          Serial.println("Subtype:     REMOVE_TEST_ACK");
          break;
        case MGMT_REMOVE_POWER_OFF:
          Serial.println("Subtype:     REMOVE_POWER_OFF");
          break;
        case MGMT_REMOVE_NOT_POSSIBLE: {
          Serial.println("Subtype:     REMOVE_NOT_POSSIBLE");
          RemoveNotPossiblePayload p; extractRemoveNotPossiblePayload(pkt, p);
          Serial.printf("Orig UP:     %d\n", p.originalUpstream);
          Serial.printf("Orig DOWN:   %d\n", p.originalDownstream);
          break;
        }
        case MGMT_REMOVE_SUCCESS:
          Serial.println("Subtype:     REMOVE_SUCCESS");
          break;
        case MGMT_TIMESYNC_REQUEST: {
          Serial.println("Subtype:     TIMESYNC_REQUEST");
          TimeSyncRequestPayload p; extractTimeSyncRequestPayload(pkt, p);
          Serial.printf("t1:          %llu us\n", p.t1);
          break;
        }
        case MGMT_TIMESYNC_RESPONSE: {
          Serial.println("Subtype:     TIMESYNC_RESPONSE");
          TimeSyncResponsePayload p; extractTimeSyncResponsePayload(pkt, p);
          Serial.printf("t2:          %llu us\n", p.t2);
          Serial.printf("t3:          %llu us\n", p.t3);
          break;
        }
        case MGMT_TIMESYNC_START:
          Serial.println("Subtype:     TIMESYNC_START");
          break;
        case MGMT_TIMESYNC_DONE:
          Serial.println("Subtype:     TIMESYNC_DONE");
          break;
        case MGMT_TDMA_SLOT: {
          Serial.println("Subtype:     TDMA_SLOT");
          TdmaSlotPayload p; extractTdmaSlotPayload(pkt, p);
          Serial.printf("Slot:        %d\n", p.slot);
          break;
        }
        default:
          Serial.printf("Subtype:     UNKNOWN (0x%02X)\n", mgmt.subtype);
          break;
      }
      break;
    }
    case PKT_TYPE_TIMESYNC:
      Serial.println("Msg Type:    TIMESYNC (0x03)");
      break;
    default:
      Serial.printf("Msg Type:    UNKNOWN (0x%02X)\n", pkt.msgType);
      break;
  }

  Serial.printf("CRC-16:      0x%04X\n", pkt.crc);
  Serial.println("===================");
}