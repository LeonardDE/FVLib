#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <fstream>

#include "Vector3.h"
#include "Path.h"
#include "lib/plane.h"

#include "json.hpp"
using json = nlohmann::json;


class GlobalSituation;

class FV
{
public:

    virtual Plane getPlane() = 0;
    virtual void next(double h, double end_time) = 0;
    virtual void setPath(vector<Point> path);
    vector<Point> getBasePath()
    {
        return basePath;
    };
    Path getDynamicPath()
    {
        return turnPath;
    };
    double getTime()
    {
        return time;
    }

    //void writeFVState(double timeStep);
    friend void mergeShortPlans(const vector<Point>& arr1, const vector<Point>& arr2,
        vector<Point>& res1, vector<Point>& res2);
    json conflicts;
protected:
    string name;
    vector<Point> basePath;
    vector<Point> dynamicPath;
    Path turnPath;
    //double maxAcceleration;
    double time = 0;

    double radiusFilter = 0;
    double broadcastStep;
    double nextBroadcastInstant = 0;

    double radiusWarn;
    double heightWarn;

    virtual double solveTurnRadius(const Vector3& v1, const Vector3& v2) =0;

    virtual void doBroadcast();

    GlobalSituation* globalSituation;

    virtual void checkConflict();
 

    
};





void mergeShortPlans(const vector<Point>& arr1, const vector<Point>& arr2,
    vector<Point>& res1, vector<Point>& res2);