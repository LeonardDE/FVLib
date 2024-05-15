#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include "Vector3.h"
#include "lib/plane.h"


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

protected:
    vector<Point> basePath;
    Path dynamicPath;
    double maxAcceleration;
    double time = 0;
};



class MaterialPoint : public FV
{
public:
    MaterialPoint(string name,
        double x, double y, double z,
        double speedX, double speedY, double speedZ,
        double maxAcceleration, double k_x, double k_v);
    ~MaterialPoint();
    Plane getPlane() override;

    void next(double h, double end_time) override;

    string getName();
    int32_t getType();
    Vector3 acceleration;
    Vector3 wishPosition;
    Vector3 wishVelocity;
private:
    string name;
    int32_t type;

    Vector3* curPosition;
    Vector3* newPosition;
    Vector3* curVelocity;
    Vector3* newVelocity;

    double k_v;
    double k_x;

    void computeWishData(double time_solve);
};

class Copter : public FV
{
public:
    Copter(string name,double x, double y, double z,
        double speedX, double speedY, double speedZ,
        double maxAcceleration,
        double inertialXZ, double inertialY,double k_x, double k_v);
    ~Copter();
    Plane getPlane() override;
    void next(double h, double end_time) override;



    Vector3 wishVelocity;
    Vector3 wishPosition;
private:
    string name;
    int32_t type;
    double inertialXZ;
    double inertialY;
    double k_x;
    double k_v;
    Vector3* curPosition;
    Vector3* newPosition;
    Vector3* curVelocity;
    Vector3* newVelocity;

    void computeWishData(double time_solve);
};

