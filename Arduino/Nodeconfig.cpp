#include "NodeConfig.h"
#include <Arduino.h>

NodeConfig Node;

NodeConfig::NodeConfig() {
  nodeId     = NODE_ID_UNSET;
  upstream   = NODE_NEIGHBOR_NONE;
  downstream = NODE_NEIGHBOR_NONE;
  enrolled   = false;
  tdmaSlot   = 0;
}

void NodeConfig::begin() {
  prefs.begin("nodeconfig", false);
  nodeId     = prefs.getUChar("nodeid",     NODE_ID_UNSET);
  upstream   = prefs.getUChar("upstream",   NODE_NEIGHBOR_NONE);
  downstream = prefs.getUChar("downstream", NODE_NEIGHBOR_NONE);
  enrolled   = prefs.getBool("enrolled",    false);
  tdmaSlot   = prefs.getUChar("tdmaslot",   0);
  prefs.end();
  Serial.println("[NodeConfig] Loaded from flash:");
  printConfig();
}

void NodeConfig::setNodeId(uint8_t id) {
  nodeId = id;
  prefs.begin("nodeconfig", false);
  prefs.putUChar("nodeid", id);
  prefs.end();
  Serial.printf("[NodeConfig] Node ID set to %d\n", id);
}

void NodeConfig::setUpstream(uint8_t id) {
  upstream = id;
  prefs.begin("nodeconfig", false);
  prefs.putUChar("upstream", id);
  prefs.end();
  Serial.printf("[NodeConfig] Upstream set to %d\n", id);
}

void NodeConfig::setDownstream(uint8_t id) {
  downstream = id;
  prefs.begin("nodeconfig", false);
  prefs.putUChar("downstream", id);
  prefs.end();
  Serial.printf("[NodeConfig] Downstream set to %d\n", id);
}

void NodeConfig::setEnrolled(bool value) {
  enrolled = value;
  prefs.begin("nodeconfig", false);
  prefs.putBool("enrolled", value);
  prefs.end();
  Serial.printf("[NodeConfig] Enrolled set to %s\n", value ? "true" : "false");
}

uint8_t NodeConfig::getNodeId()     const { return nodeId; }
uint8_t NodeConfig::getUpstream()   const { return upstream; }
uint8_t NodeConfig::getDownstream() const { return downstream; }
bool    NodeConfig::isMaster()      const { return nodeId == NODE_ID_MASTER; }
bool    NodeConfig::isEnrolled()    const { return enrolled; }
uint8_t NodeConfig::getTdmaSlot()   const { return tdmaSlot; }

void NodeConfig::setTdmaSlot(uint8_t slot) {
  tdmaSlot = slot;
  prefs.begin("nodeconfig", false);
  prefs.putUChar("tdmaslot", slot);
  prefs.end();
  Serial.printf("[NodeConfig] TDMA slot set to %d\n", slot);
}

uint8_t NodeConfig::resolveNextHop(uint8_t destination) const {
  if (nodeId == NODE_ID_UNSET) {
    Serial.println("[NodeConfig] ERROR: Node ID not set");
    return NODE_NEIGHBOR_NONE;
  }
  if (destination == nodeId) {
    Serial.println("[NodeConfig] WARNING: Destination is self");
    return NODE_NEIGHBOR_NONE;
  }
  if (isMaster()) {
    if (downstream == NODE_NEIGHBOR_NONE) {
      Serial.println("[NodeConfig] ERROR: Master has no downstream neighbor");
      return NODE_NEIGHBOR_NONE;
    }
    return downstream;
  }
  if (destination == upstream) {
    if (upstream == NODE_NEIGHBOR_NONE) {
      Serial.println("[NodeConfig] ERROR: No upstream neighbor set");
      return NODE_NEIGHBOR_NONE;
    }
    return upstream;
  }
  if (destination == downstream) {
    if (downstream == NODE_NEIGHBOR_NONE) {
      Serial.println("[NodeConfig] ERROR: No downstream neighbor set");
      return NODE_NEIGHBOR_NONE;
    }
    return downstream;
  }
  if (destination == NODE_ID_MASTER) {
    if (upstream == NODE_NEIGHBOR_NONE) {
      Serial.println("[NodeConfig] ERROR: No upstream neighbor set");
      return NODE_NEIGHBOR_NONE;
    }
    return upstream;
  }
  if (downstream == NODE_NEIGHBOR_NONE) {
    Serial.println("[NodeConfig] ERROR: No downstream neighbor set");
    return NODE_NEIGHBOR_NONE;
  }
  return downstream;
}

void NodeConfig::printConfig() const {
  Serial.println("=== Node Configuration ===");
  if (nodeId == NODE_ID_UNSET)
    Serial.println("Node ID:    NOT SET");
  else
    Serial.printf("Node ID:    %d%s\n", nodeId, isMaster() ? " (MASTER)" : "");
  if (upstream == NODE_NEIGHBOR_NONE) Serial.println("Upstream:   NONE");
  else Serial.printf("Upstream:   %d\n", upstream);
  if (downstream == NODE_NEIGHBOR_NONE) Serial.println("Downstream: NONE");
  else Serial.printf("Downstream: %d\n", downstream);
  Serial.printf("Enrolled:   %s\n", enrolled ? "YES" : "NO");
  Serial.println("==========================");
}