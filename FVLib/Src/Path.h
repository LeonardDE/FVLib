#pragma once
#include <vector>
#include "PathPoint.h"

class Path {
public:
  Path(vector<PathPoint> path = vector<PathPoint>{}) : path(path) {};

  PathPoint getPointForIndex(int ind);
  void addNewPoint(PathPoint p);
  vector<PathPoint> getPath();

private:
  vector<PathPoint> path;
};
