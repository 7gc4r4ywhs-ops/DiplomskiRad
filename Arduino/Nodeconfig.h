#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>

#define NODE_ID_MASTER      0
#define NODE_ID_UNSET       255
#define NODE_NEIGHBOR_NONE  255

class NodeConfig {
public:
  NodeConfig();

  void begin();

  void setNodeId(uint8_t id);
  void setTdmaSlot(uint8_t slot);
  void setUpstream(uint8_t id);
  void setDownstream(uint8_t id);
  void setEnrolled(bool value);

  uint8_t getNodeId()     const;
  uint8_t getTdmaSlot()   const;
  uint8_t getUpstream()   const;
  uint8_t getDownstream() const;
  bool    isMaster()      const;
  bool    isEnrolled()    const;

  uint8_t resolveNextHop(uint8_t destination) const;

  void printConfig() const;

private:
  Preferences prefs;
  uint8_t nodeId;
  uint8_t upstream;
  uint8_t downstream;
  bool    enrolled;
  uint8_t tdmaSlot;  // 1, 2, or 3 (0 = not assigned)
};

extern NodeConfig Node;

#endif