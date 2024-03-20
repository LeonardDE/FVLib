#include "flight_plan.h"
#include "plane.h"
#include "vector"

#ifndef DRONE_LIB_SIMULATION_H
#define DRONE_LIB_SIMULATION_H

class Simulation
{
private:
    const Scheme scheme;

    vector<StateObserver> stateObservers;

    static Scheme loadScheme();

    void notifyObservers();

public:
    Simulation();

    ~Simulation();

    Scheme getScheme();

    void subscribeAirState(StateObserver observer);

    void setTime(int32_t time);

    void start();

    void pause();

    void stop();
};

#endif //DRONE_LIB_SIMULATION_H
