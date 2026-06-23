#ifndef NETWORK_TOPOLOGY_H
#define NETWORK_TOPOLOGY_H

#include <Arduino.h>
#include <Preferences.h>

#define TOPOLOGY_MAX_NODES 16
#define TOPOLOGY_NODE_NONE 255

// Single node entry in the topology table
struct NodeEntry {
  uint8_t nodeId;       // Node ID (TOPOLOGY_NODE_NONE = empty slot)
  uint8_t upstreamId;   // Upstream neighbor
  uint8_t downstreamId; // Downstream neighbor (TOPOLOGY_NODE_NONE if end of chain)
};

class NetworkTopology {
public:
  NetworkTopology();

  // Load topology from flash. Call in setup() on master.
  void begin();

  // Add a new node or update existing entry. Persists to flash.
  void addOrUpdateNode(uint8_t nodeId, uint8_t upstreamId, uint8_t downstreamId);

  // Remove a node entry. Persists to flash.
  void removeNode(uint8_t nodeId);

  // Get a node entry by ID. Returns false if not found.
  bool getNode(uint8_t nodeId, NodeEntry& out) const;

  // Number of enrolled nodes currently stored
  uint8_t getNodeCount() const;

  // Print full topology to Serial
  void printTopology() const;

private:
  NodeEntry nodes[TOPOLOGY_MAX_NODES];
  uint8_t   nodeCount;
  Preferences prefs;

  // Find index of a node by ID, returns -1 if not found
  int findIndex(uint8_t nodeId) const;

  // Persist entire table to flash
  void saveToFlash();
};

extern NetworkTopology Topology;

#endif