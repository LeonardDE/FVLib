#include "Path.h"

PathPoint Path::getPointForIndex(int ind)
{
    return path[ind];
};

void Path::addNewPoint(PathPoint p)
{
    path.push_back(p);
}


vector<PathPoint> Path::getPath()
{
    return path;
}