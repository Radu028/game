#pragma once

#include <memory>
#include <vector>

#include "raylib.h"

// Navigation node for A* pathfinding
struct NavNode {
  Vector3 position;
  std::vector<int> connections;  // Indices of connected nodes
  bool walkable = true;
  float gCost = 0.0f;  // Cost from start
  float hCost = 0.0f;  // Cost to end (heuristic)
  float fCost = 0.0f;  // Total cost (g + h)
  int parent = -1;     // Parent node index for path reconstruction

  NavNode(Vector3 pos) : position(pos) {}
};

// Navigation mesh for pathfinding
class NavMesh {
 private:
  std::vector<NavNode> nodes;
  float nodeSpacing;
  Vector3 minBounds;
  Vector3 maxBounds;

 public:
  NavMesh(Vector3 minBounds, Vector3 maxBounds, float spacing = 1.0f);

  // Setup
  void generateNavMesh();
  void rebuildConnections();

  // Obstacle management system
  void addObstacle(Vector3 position, Vector3 size,
                   const std::string& type = "generic",
                   bool ignoreYAxis = false);
  void addShelfObstacle(Vector3 shelfPos, Vector3 shelfSize);
  void addWallObstacle(Vector3 wallPos, Vector3 wallSize);
  void removeObstacle(Vector3 position, Vector3 size);
  void markNodesInArea(Vector3 center, Vector3 size, bool walkable,
                       float expansionFactor = 0.5f,
                       bool ignoreYAxis = false);
  bool isPositionBlocked(Vector3 position) const;

  void defineShopEntrance(Vector3 entrancePos, Vector3 entranceSize);
  void defineShopInterior(Vector3 shopPos, Vector3 shopSize);

  // Pathfinding
  std::vector<Vector3> findPath(Vector3 start, Vector3 end);
  std::vector<Vector3> findAlternativePath(Vector3 start, Vector3 end,
                                           Vector3 blockedArea);
  int findNearestNode(Vector3 position) const;
  bool isWalkable(Vector3 position) const;

  // A* algorithm implementation
  std::vector<int> aStar(int startNode, int endNode);

  // Utility
  float calculateDistance(int nodeA, int nodeB) const;
  float calculateHeuristic(int nodeA, int nodeB) const;
  void connectNodes();
  bool hasLineOfSight(Vector3 from, Vector3 to) const;
  bool isNodeFullyAccessible(Vector3 nodePos, Vector3 entrancePos,
                             Vector3 entranceSize) const;
  bool isNodeTooCloseToWall(Vector3 nodePos, Vector3 entrancePos,
                            Vector3 entranceSize) const;

  // Debug
  void debugDraw() const;
  void debugDrawEntranceNodes(Vector3 entrancePos, Vector3 entranceSize) const;
  const std::vector<NavNode>& getNodes() const { return nodes; }
};
