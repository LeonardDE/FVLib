#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <fstream>

#include "Vector3.h"
#include "lib/plane.h"
#include "GlobalSituation.h"

struct TurnData
{
    TurnData(
        double w = 0, double eps = 0, double w_0=0,
        Vector3 axis=Vector3().Zero(), 
        Vector3 axisPoint = Vector3().Zero()) : 
            angularVelocity(w), angularAcceleration(eps), angularVelocity_0(w_0), axis(axis), axisPoint(axisPoint) {};
    
    double angularVelocity;
    double angularAcceleration;
    Vector3 axis;
    Vector3 axisPoint;

    double angularVelocity_0;
};

enum class PointType
{
    DEFAULT = 0,
    START_TURN = 1
};

struct Point
{
    Point(Vector3 position = Vector3(0, 0, 0), double arrivalTime=0) : position(position), arrivalTime(arrivalTime) {};

    Vector3 position;
    double arrivalTime;
};

struct PathPoint : public Point
{
    PathPoint(
        Vector3 position = Vector3(0, 0, 0),
        double arrivalTime = 0,
        PointType type = PointType::DEFAULT,TurnData turnData=TurnData()) : 
            position(position), arrivalTime(arrivalTime), type(type), turnData(turnData) {};
    
    PathPoint(
        Point point = Point(),
        PointType type = PointType::DEFAULT,
        TurnData turnData = TurnData()) : position(point.position), arrivalTime(point.arrivalTime), type(type), turnData(turnData) {};

    Vector3 position;
    double arrivalTime;

    PointType type;
    TurnData turnData;

};

class Path
{
public:
    Path(vector<PathPoint> path= vector<PathPoint>{}, int index=0) : path(path) {};
    

    PathPoint getPointForIndex(int ind)
    {
        return path[ind];
    };

    void addNewPoint(PathPoint p)
    {
        path.push_back(p);
    }


    vector<PathPoint> getPath()
    {
        return path;
    }

private:
    vector<PathPoint> path;
};


class FV
{
public:

    virtual Plane getPlane() abstract;
    virtual void next(double h, double end_time) abstract;
    virtual void setPath(vector<Point> path);
    vector<Point> getBasePath()
    {
        return basePath;
    };
    Path getDynamicPath()
    {
        return dynamicPath;
    };
    double getTime()
    {
        return time;
    }

    //void writeFVState(double timeStep);

protected:
    string name;
    vector<Point> basePath;
    Path dynamicPath;
    //double maxAcceleration;
    double time = 0;

    double radiusFilter = 0;
    double broadcastStep;
    double nextBroadcastInstant = 0;

    double radiusWarn;

    virtual double solveTurnRadius(const Vector3& v1, const Vector3& v2) =0;

    GlobalSituation* globalSituation;

    virtual void checkConflict();

    friend void mergeShortPlans(const vector<Point>& arr1, const vector<Point>& arr2,
                                      vector<Point>& res1,       vector<Point>& res2);
};



class MaterialPoint : public FV
{
public:
    MaterialPoint(string name,
        double x, double y, double z,
        double speedX, double speedY, double speedZ,
        double maxAcceleration, double k_x, double k_v,
        double writeTime, GlobalSituation* gs);
    ~MaterialPoint();
    Plane getPlane() override;

    void next(double h, double end_time) override;

    string getName();
    int32_t getType();
    // текущее ускорение 
    Vector3 acceleration;
    
    Vector3 wishPosition;
    Vector3 wishVelocity;
private:
    int32_t type;

    Vector3* curPosition;
    Vector3* newPosition;
    Vector3* curVelocity;
    Vector3* newVelocity;

    double maxAcceleration;

    double k_v;
    double k_x;

    double solveTurnRadius(const Vector3& v1, const Vector3& v2) override;

    void computeWishData(double time_solve);
};

class Copter : public FV
{
public:
    Copter(const string& name,double x, double y, double z,
        double speedX, double speedY, double speedZ,
        double inertialXZ, double inertialY,double k_x, double k_v,
        double maxVelocity_xz,double maxVelocity_y,double minVelocity_y,
        double writeTime, GlobalSituation* gs);
    ~Copter();
    Plane getPlane() override;
    void next(double h, double end_time) override;



    Vector3 wishVelocity;
    Vector3 wishPosition;
private:
    int32_t type;
    double inertialXZ;
    double inertialY;
    double k_x;
    double k_v;
    Vector3* curPosition;
    Vector3* newPosition;
    Vector3* curVelocity;
    Vector3* newVelocity;

    double maxVelocity_xz;
    double maxVelocity_y;
    double minVelocity_y;

    double solveTurnRadius(const Vector3& v1, const Vector3& v2) override;

    void computeWishData(double time_solve);
};



// слияние Путей

///
void mergeShortPlans(const vector<Point>& arr1, const vector<Point>& arr2,
                vector<Point>& res1 , vector<Point>& res2) 
{
    res1.clear();
    res2.clear();

    size_t i = 0;
    size_t j = 0;

    double maxStart = max(arr1[0].arrivalTime,arr2[0].arrivalTime);
    double minEnd = min(arr2[arr2.size() - 1].arrivalTime, arr1[arr1.size() - 1].arrivalTime);

    while (i < arr1.size() && j < arr2.size()) {
        if (arr1[i].arrivalTime < arr2[j].arrivalTime) {
            if (arr1[i].arrivalTime >= maxStart && arr1[i].arrivalTime <= minEnd)
            {
                Point p;
                p.arrivalTime = arr1[i].arrivalTime;
                p.position = arr2[j].position + (arr2[j + 1].position - arr2[j].position)
                    * (p.arrivalTime - arr2[j].arrivalTime)
                    / (arr2[j + 1].arrivalTime - arr2[j].arrivalTime);
                res2.push_back(p);
                res1.push_back(arr1[i]);
            }
            i++;
        }
        else if (arr1[i].arrivalTime > arr2[j].arrivalTime) {
            if (arr2[j].arrivalTime >= maxStart && arr2[j].arrivalTime <= minEnd)
            {
                Point p;
                p.arrivalTime = arr2[j].arrivalTime;
                p.position = arr1[i].position + (arr1[i + 1].position - arr1[i].position)
                    * (p.arrivalTime - arr1[i].arrivalTime)
                    / (arr1[i + 1].arrivalTime - arr1[i].arrivalTime);
                res1.push_back(p);
                res2.push_back(arr2[j]);
            }
            j++;
        }
        else
        {
            res1.push_back(arr1[i]);
            res2.push_back(arr2[j]);
            i++;
            j++;
        }
    }
}