#include "path.h"
using namespace HybridAStar;

void Path::clear() {
  Node3D node;
  path.pathpoints.clear();
}

void Path::updatePath(std::vector<Node3D> nodePath) {
  path.header.stamp = ros::Time::now();
  geometry_msgs::Vector3 vertex;
  vertex.z = 0;
  for (int i = 0; i < nodePath.size(); ++i) {
    vertex.x = nodePath[i].getX() * Constants::cellSize;
    vertex.y = nodePath[i].getY() * Constants::cellSize;
    path.pathpoints.push_back(vertex);
  }
  return;
}
