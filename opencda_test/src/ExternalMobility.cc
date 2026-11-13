#include "ExternalMobility.h"

#include "inet/common/InitStages.h"

Define_Module(ExternalMobility);

void ExternalMobility::initialize(int stage)
{
    inet::MovingMobilityBase::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL)
        stationary = false;
}

void ExternalMobility::move()
{
    // The lockstep coordinator supplies position and velocity explicitly.
}

void ExternalMobility::setExternalState(const inet::Coord& position, const inet::Coord& velocity)
{    Enter_Method_Silent();
    lastPosition = position;
    lastVelocity = velocity;
    lastUpdate = omnetpp::simTime();
    emitMobilityStateChangedSignal();
}

