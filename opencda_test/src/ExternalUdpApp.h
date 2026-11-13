/*
 * ExternalUdpApp.h
 *
 *  Created on: Oct 2, 2026
 *      Author: ansari
 */

#ifndef EXTERNALUDPAPP_H_
#define EXTERNALUDPAPP_H_

#include <omnetpp.h>

#include "inet/applications/base/ApplicationBase.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"

class CarlaCoordinator;

class ExternalUdpApp : public inet::ApplicationBase
{
  protected:
    inet::UdpSocket socket;
    //inet::L3Address destinationAddress;

    int localPort = -1;
    //int destinationPort = -1;
    std::string actorName;
    CarlaCoordinator *coordinator = nullptr;

    virtual int numInitStages() const override;
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(omnetpp::cMessage *message) override;

    virtual void handleStartOperation(
        inet::LifecycleOperation *operation) override;

    virtual void handleStopOperation(
        inet::LifecycleOperation *operation) override;

    virtual void handleCrashOperation(
        inet::LifecycleOperation *operation) override;

  public:
    void sendExternalMessage(
        long messageId,
        int payloadSize,
        const inet::L3Address& destination,
        int destinationPort
    );
};




#endif /* EXTERNALUDPAPP_H_ */
