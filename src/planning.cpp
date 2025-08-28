#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

// Structure for A* algorithm nodes
struct Node {
    int x, y;
    double g, h, f;  // Fixed order: g, h, f (f = g + h)
    pair<int, int> parent;
    
    Node(int x, int y, double g, double h, pair<int, int> parent) 
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}
        
    // For priority queue (min-heap based on f value)
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  vector<pair<int, int>> path; // store final path 
  
  // Check if start and goal are valid
  if (!isvalid(start.first, start.second) || !isvalid(goal.first, goal.second)) {
    cout << "Start or goal position is invalid!" << endl;
    return path;
  }
  
  // If start is goal, return path with just start
  if (start == goal) {
    path.push_back(start);
    return path;
  }
  
  // Priority queue for open set (min-heap)
  priority_queue<Node, vector<Node>, greater<Node>> openSet;
  
  // Maps to track visited nodes and their costs
  map<pair<int, int>, double> gScore;
  map<pair<int, int>, pair<int, int>> cameFrom;
  
  // 8-directional movement (including diagonals)
  vector<pair<int, int>> directions = {
    {-1, -1}, {-1, 0}, {-1, 1},
    {0, -1},           {0, 1},
    {1, -1},  {1, 0},  {1, 1}
  };
  
  // Initialize starting node
  double h_start = heuristic(start.first, start.second, goal.first, goal.second);
  openSet.push(Node(start.first, start.second, 0.0, h_start, {-1, -1}));
  gScore[start] = 0.0;
  
  while (!openSet.empty()) {
    Node current = openSet.top();
    openSet.pop();
    
    pair<int, int> currentPos = {current.x, current.y};
    
    // Check if we reached the goal
    if (currentPos == goal) {
      // Reconstruct path
      pair<int, int> pos = goal;
      while (pos != make_pair(-1, -1)) {
        path.push_back(pos);
        pos = cameFrom[pos];
      }
      reverse(path.begin(), path.end());
      return path;
    }
    
    // Skip if we've already processed this node with a better cost
    if (gScore.find(currentPos) != gScore.end() && current.g > gScore[currentPos]) {
      continue;
    }
    
    // Explore neighbors
    for (auto& dir : directions) {
      int newX = current.x + dir.first;
      int newY = current.y + dir.second;
      pair<int, int> neighbor = {newX, newY};
      
      // Check if neighbor is valid
      if (!isvalid(newX, newY)) {
        continue;
      }
      
      // Calculate movement cost (diagonal moves cost more)
      double moveCost = (dir.first != 0 && dir.second != 0) ? sqrt(2.0) : 1.0;
      double tentativeG = current.g + moveCost;
      
      // Skip if we've found a better path to this neighbor
      if (gScore.find(neighbor) != gScore.end() && tentativeG >= gScore[neighbor]) {
        continue;
      }
      
      // Record the best path to this neighbor
      cameFrom[neighbor] = currentPos;
      gScore[neighbor] = tentativeG;
      
      // Add neighbor to open set
      double h = heuristic(newX, newY, goal.first, goal.second);
      openSet.push(Node(newX, newY, tentativeG, h, currentPos));
    }
  }
  
  // No path found
  cout << "No path found from start to goal!" << endl;
  return path;
}