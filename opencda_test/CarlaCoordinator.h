

#ifndef __OPENCDA_SIMU5G_CARLACOORDINATOR_H
#define __OPENCDA_SIMU5G_CARLACOORDINATOR_H

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "json.hpp"
#include <omnetpp.h>
#include <zmq.hpp>

#include "ExternalMobility.h"

// Forward declaration goes here, before CarlaCoordinator.
class ExternalUdpApp;

class CarlaCoordinator : public omnetpp::cSimpleModule
{
  protected:
    // Existing ZeroMQ members
    std::unique_ptr<zmq::context_t> context;
    std::unique_ptr<zmq::socket_t> socket;

    // Existing simulation members
    omnetpp::cMessage *stepEvent = nullptr;
    std::vector<ExternalMobility *> mobilities;
    std::vector<ExternalUdpApp *> udpApps;

    std::map<std::string, int> actorToUe;

    // Existing mobility log
    std::ofstream log;

    // Add the message log here
    std::ofstream messageLog;

    // Messages received by Simu5G and waiting to be
    // delivered to Python during the next STEP request
    nlohmann::json deliveredMessages =
        nlohmann::json::array();

    // Messages sent by Python but not yet received back
    std::map<long, nlohmann::json> inFlightMessages;

    // Existing parameters
    omnetpp::simtime_t stepSize;
    double duration = 0;
    int seed = 0;
    double xOffset = 0;
    double yOffset = 0;
    double zOffset = 0;
    bool invertY = true;
    bool initialized = false;
    bool remoteStopped = false;

    // Existing OMNeT++ functions
    virtual int numInitStages() const override;
    virtual void initialize(int stage) override;
    virtual void handleMessage(
        omnetpp::cMessage *message) override;
    virtual void finish() override;

    // Existing bridge functions
    nlohmann::json exchange(
        const nlohmann::json& request);

    void applyState(
        const nlohmann::json& state);

    void stopRemote() noexcept;

    // Add this function here
    void injectOutgoingMessages(
        const nlohmann::json& state);

  public:
    // ExternalUdpApp must be able to call this
    void recordDelivery(
        long messageId,
        const std::string& receiverActor,
        double networkReceiveTime
    );

    virtual ~CarlaCoordinator();
};

#endif
