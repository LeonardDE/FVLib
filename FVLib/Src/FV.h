#pragma once
#include <iostream>
#include <vector>
#include <map>
#include "lib/plane.h"

class FV abstract
{
public:
    //FV(const double& x, const double& y, const double& z, const double& speedX, const double& speedY, const double& speedZ);
    //~FV();
    virtual Plane getPlane() abstract;
    // Ускорение убрать, добавить шаг , map<string,void*> &dict | hashmap<string,void*> 
    virtual void next(double h, double end_time, map<string, void*>& dict) abstract;
    
protected:
    double time = 0;
    //vector<double>* curPosition;
    //vector<double>* newPosition;
    //vector<double>* curVelocity;
    //vector<double>* newVelocity;
};

class MaterialPoint : public FV
{
public:
    MaterialPoint(string name,
        const double& x, const double& y, const double& z,
        const double& speedX, const double& speedY, const double& speedZ);
    ~MaterialPoint();
    Plane getPlane() override;
    void next(double h, double end_time, map<string, void*>& dict) override;

    string getName();
    int32_t getType();
private:
    string name;
    int32_t type;
    vector<double>* curPosition;
    vector<double>* newPosition;
    vector<double>* curVelocity;
    vector<double>* newVelocity;
};

class Copter : public FV
{
public:
    Copter(string name,
        const double& x, const double& y, const double& z,
        const double& speedX, const double& speedY, const double& speedZ,
        const double& wishSpeedX, const double& wishSpeedY, const double& wishSpeedZ);
    ~Copter();
    Plane getPlane() override;
    void next(double h, double end_time, map<string, void*>& dict) override;
private:
    string name;
    int32_t type;
    vector<double>* wishVelocity;
    vector<double>* curPosition;
    vector<double>* newPosition;
    vector<double>* curVelocity;
    vector<double>* newVelocity;
};

