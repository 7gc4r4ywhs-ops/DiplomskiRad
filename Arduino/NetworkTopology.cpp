#include "NetworkTopology.h"
#include <Arduino.h>

NetworkTopology Topology;

NetworkTopology::NetworkTopology() {
  nodeCount = 0;
  for (int i = 0; i < TOPOLOGY_MAX_NODES; i++) {
    nodes[i].nodeId       = TOPOLOGY_NODE_NONE;
    nodes[i].upstreamId   = TOPOLOGY_NODE_NONE;
    nodes[i].downstreamId = TOPOLOGY_NODE_NONE;
  }
}

void NetworkTopology::begin() {
  prefs.begin("topology", false);
  nodeCount = prefs.getUChar("count", 0);

  for (uint8_t i = 0; i < nodeCount && i < TOPOLOGY_MAX_NODES; i++) {
    String keyId   = "n" + String(i) + "id";
    String keyUp   = "n" + String(i) + "up";
    String keyDown = "n" + String(i) + "dn";
    nodes[i].nodeId       = prefs.getUChar(keyId.c_str(),   TOPOLOGY_NODE_NONE);
    nodes[i].upstreamId   = prefs.getUChar(keyUp.c_str(),   TOPOLOGY_NODE_NONE);
    nodes[i].downstreamId = prefs.getUChar(keyDown.c_str(), TOPOLOGY_NODE_NONE);
  }

  prefs.end();
  Serial.printf("[Topology] Loaded %d nodes from flash\n", nodeCount);
  printTopology();
}

void NetworkTopology::addOrUpdateNode(uint8_t nodeId, uint8_t upstreamId, uint8_t downstreamId) {
  int idx = findIndex(nodeId);

  if (idx >= 0) {
    // Update existing
    nodes[idx].upstreamId   = upstreamId;
    nodes[idx].downstreamId = downstreamId;
    Serial.printf("[Topology] Updated node %d (up=%d, down=%d)\n",
                  nodeId, upstreamId, downstreamId);
  } else {
    // Add new
    if (nodeCount >= TOPOLOGY_MAX_NODES) {
      Serial.println("[Topology] ERROR: Max nodes reached, cannot add more");
      return;
    }
    nodes[nodeCount].nodeId       = nodeId;
    nodes[nodeCount].upstreamId   = upstreamId;
    nodes[nodeCount].downstreamId = downstreamId;
    nodeCount++;
    Serial.printf("[Topology] Added node %d (up=%d, down=%d), total=%d\n",
                  nodeId, upstreamId, downstreamId, nodeCount);
  }

  saveToFlash();
}

void NetworkTopology::removeNode(uint8_t nodeId) {
  int idx = findIndex(nodeId);
  if (idx < 0) {
    Serial.printf("[Topology] Node %d not found\n", nodeId);
    return;
  }

  // Shift remaining entries down
  for (int i = idx; i < nodeCount - 1; i++) {
    nodes[i] = nodes[i + 1];
  }
  nodes[nodeCount - 1].nodeId       = TOPOLOGY_NODE_NONE;
  nodes[nodeCount - 1].upstreamId   = TOPOLOGY_NODE_NONE;
  nodes[nodeCount - 1].downstreamId = TOPOLOGY_NODE_NONE;
  nodeCount--;

  Serial.printf("[Topology] Removed node %d, total=%d\n", nodeId, nodeCount);
  saveToFlash();
}

bool NetworkTopology::getNode(uint8_t nodeId, NodeEntry& out) const {
  int idx = findIndex(nodeId);
  if (idx < 0) return false;
  out = nodes[idx];
  return true;
}

uint8_t NetworkTopology::getNodeCount() const {
  return nodeCount;
}

void NetworkTopology::printTopology() const {
  Serial.println("=== Network Topology ===");
  Serial.printf("Nodes: %d / %d\n", nodeCount, TOPOLOGY_MAX_NODES);
  Serial.println("ID   UP   DOWN");
  for (uint8_t i = 0; i < nodeCount; i++) {
    String down = nodes[i].downstreamId == TOPOLOGY_NODE_NONE
                  ? "none"
                  : String(nodes[i].downstreamId);
    Serial.printf("%-4d %-4d %s\n", nodes[i].nodeId, nodes[i].upstreamId, down.c_str());
  }
  Serial.println("========================");
}

// ── Private ───────────────────────────────────────────────────────────────────

int NetworkTopology::findIndex(uint8_t nodeId) const {
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].nodeId == nodeId) return i;
  }
  return -1;
}

void NetworkTopology::saveToFlash() {
  prefs.begin("topology", false);
  prefs.putUChar("count", nodeCount);

  for (uint8_t i = 0; i < nodeCount; i++) {
    String keyId   = "n" + String(i) + "id";
    String keyUp   = "n" + String(i) + "up";
    String keyDown = "n" + String(i) + "dn";
    prefs.putUChar(keyId.c_str(),   nodes[i].nodeId);
    prefs.putUChar(keyUp.c_str(),   nodes[i].upstreamId);
    prefs.putUChar(keyDown.c_str(), nodes[i].downstreamId);
  }

  prefs.end();
}