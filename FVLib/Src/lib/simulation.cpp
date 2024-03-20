#include "simulation.h"

Simulation::Simulation() : scheme(loadScheme())
{}

Simulation::~Simulation() = default;

void Simulation::subscribeAirState(StateObserver observer) {
    stateObservers.insert(stateObservers.end(), observer);
}

void Simulation::setTime(int32_t time) {
    // TODO
}

void Simulation::start() {
    // TODO
}

void Simulation::pause() {
    // TODO
}

void Simulation::stop() {
    // TODO
}


Scheme Simulation::loadScheme() {
    Scheme result;
    return result;
}

//void Simulation::notifyObservers() {
//    std::for_each(
//        stateObservers.begin(),
//        stateObservers.end(),
//        [&](const auto& item) {
//            item(AirSituationState());
//        }
//    );
//}