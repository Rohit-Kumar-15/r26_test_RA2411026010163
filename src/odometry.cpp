#include "odometry.h"
#include <cmath>
#include <ctime>
#include <iterator>
#include <numeric>

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  // Linear velocity (m/s) =(wheel circumference * revolutions per second)
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  // atan2 returns radians, convert to degrees
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {

  MotionCommand res = {0.0, 0.0}; // store total time and angle traversed

  // Check if path is valid
  if (path.size() < 2) {
    return res; // No movement needed for empty or single-point path
  }

  double totalDistance = 0.0;
  double totalAngleChange = 0.0;
  double currentHeading = 0.0; // Initial heading (facing east, 0 degrees)
  
  // Process each segment of the path
  for (size_t i = 0; i < path.size() - 1; i++) {
    int x1 = path[i].first;
    int y1 = path[i].second;
    int x2 = path[i + 1].first;
    int y2 = path[i + 1].second;
    
    // Calculate distance for this segment
    double segmentDistance = distance(x1, y1, x2, y2);
    totalDistance += segmentDistance;
    
    // Calculate required heading for this segment
    double requiredHeading = angle(x1, y1, x2, y2);
    
    // Calculate angle change needed
    double angleChange = requiredHeading - currentHeading;
    
    // Normalize angle change to [-180, 180] range
    while (angleChange > 180.0) {
      angleChange -= 360.0;
    }
    while (angleChange < -180.0) {
      angleChange += 360.0;
    }
    
    // Add absolute angle change to total
    totalAngleChange += abs(angleChange);
    
    // Update current heading
    currentHeading = requiredHeading;
  }
  
  // Calculate total time based on distance and linear velocity
  // Each grid cell represents 1 meter (as per cellsize in gridmap)
  double totalTime = totalDistance / linear_vel;
  
  // Store results
  res.time_sec = totalTime;
  res.angle_deg = totalAngleChange;

  return res;
}