#ifndef __OPENCDA_SIMU5G_EXTERNALMOBILITY_H
#define __OPENCDA_SIMU5G_EXTERNALMOBILITY_H

#include "inet/mobility/base/MovingMobilityBase.h"

class ExternalMobility : public inet::MovingMobilityBase
{
  protected:
    virtual void initialize(int stage) override;
    virtual void move() override;

  public:
    void setExternalState(const inet::Coord& position, const inet::Coord& velocity);
};

#endif

