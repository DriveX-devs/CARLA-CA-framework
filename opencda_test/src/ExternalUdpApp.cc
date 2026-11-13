/*
 * ExternalUdpApp.cc
 *
 *  Created on: Oct 2, 2026
 *      Author: ansari
 */

#include "ExternalUdpApp.h"
#include "CarlaCoordinator.h"

#include "inet/common/InitStages.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/ByteCountChunk.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include <stdexcept>
#include <string>
Define_Module(ExternalUdpApp);

void ExternalUdpApp::initialize(int stage)
{
    inet::ApplicationBase::initialize(stage);

    if (stage == inet::INITSTAGE_APPLICATION_LAYER) {
        localPort = par("localPort");
        //destinationPort = par("destPort");

        //destinationAddress =
            //inet::L3AddressResolver().resolve(
                //par("destAddress").stringValue()
            //);
        actorName =
            par("actorName").stringValue();
        coordinator =
            dynamic_cast<CarlaCoordinator *>(
                getModuleByPath(
                    par("coordinatorModule").stringValue()
                )
            );

        if (coordinator == nullptr) {
            throw omnetpp::cRuntimeError(
                "Module '%s' was not found or is not "
                "a CarlaCoordinator",
                par("coordinatorModule").stringValue()
            );
        }

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
    }
}

void ExternalUdpApp::handleStartOperation(
    inet::LifecycleOperation *operation)
{
    // The socket is already initialized and bound in initialize().
    (void)operation;
}

void ExternalUdpApp::handleStopOperation(
    inet::LifecycleOperation *operation)
{
    (void)operation;
    socket.close();
}

void ExternalUdpApp::handleCrashOperation(
    inet::LifecycleOperation *operation)
{
    (void)operation;
    socket.destroy();
}

int ExternalUdpApp::numInitStages() const
{
    return inet::NUM_INIT_STAGES;
}

void ExternalUdpApp::sendExternalMessage(
    long messageId,
    int payloadSize,
    const inet::L3Address& destination,
    int destinationPort)
{
    Enter_Method(
        "sendExternalMessage(messageId=%ld)",
        messageId
    );

    const std::string packetName =
        "OpenCda-"
        + std::to_string(messageId);

    auto *packet =
        new inet::Packet(
            packetName.c_str()
        );

    const auto& payload =
        inet::makeShared<
            inet::ByteCountChunk
        >(
            inet::B(payloadSize)
        );

    packet->insertAtBack(payload);

    socket.sendTo(
        packet,
        destination,
        destinationPort
    );
}

void ExternalUdpApp::handleMessageWhenUp(
    omnetpp::cMessage *message)
{
    auto *packet =
        dynamic_cast<inet::Packet *>(message);

    if (packet == nullptr) {
        EV_WARN << "ExternalUdpApp received a non-packet message: "
                << message->getName() << '\n';

        delete message;
        return;
    }

    const std::string packetName = packet->getName();
    const std::string prefix = "OpenCda-";

    if (packetName.compare(
            0, prefix.size(), prefix) == 0) {

        const std::string idPart =
            packetName.substr(prefix.size());

        try {
            const long messageId =
                std::stol(idPart);

            coordinator->recordDelivery(
                messageId,
                actorName,
                omnetpp::simTime().dbl()
            );

            EV_INFO
                << "Received OpenCDA packet "
                << messageId
                << " at "
                << omnetpp::simTime()
                << '\n';
        }
        catch (const std::invalid_argument&) {
            EV_WARN
                << "Cannot extract message ID from packet name: "
                << packetName
                << '\n';
        }
        catch (const std::out_of_range&) {
            EV_WARN
                << "Message ID is outside the valid range: "
                << packetName
                << '\n';
        }
    }
    else {
        EV_WARN
            << "Received packet with unexpected name: "
            << packetName
            << '\n';
    }

    delete packet;
}



