#include "CarlaCoordinator.h"
#include "ExternalUdpApp.h"
#include <algorithm>
#include <filesystem>
#include <sstream>
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/InitStages.h"

using nlohmann::json;
using namespace omnetpp;

Define_Module(CarlaCoordinator);

int CarlaCoordinator::numInitStages() const
{
    return inet::NUM_INIT_STAGES;
}

void CarlaCoordinator::initialize(int stage)
{
    if (stage != inet::INITSTAGE_LAST)
        return;

    stepSize = par("step");
    duration = par("duration").doubleValue();
    seed = par("seed");
    xOffset = par("xOffset").doubleValue();
    yOffset = par("yOffset").doubleValue();
    zOffset = par("zOffset").doubleValue();
    invertY = par("invertY");

    const int numActors = par("numActors");

    mobilities.reserve(numActors);
    udpApps.reserve(numActors);

    for (int i = 0; i < numActors; ++i) {
        const std::string mobilityPath =
            "^.ue[" + std::to_string(i)
            + "].mobility";

        const std::string appPath =
            "^.ue[" + std::to_string(i)
            + "].app[0]";

        auto *mobilityModule =
            getModuleByPath(
                mobilityPath.c_str()
            );

        auto *applicationModule =
            getModuleByPath(
                appPath.c_str()
            );

        auto *externalMobility =
            dynamic_cast<ExternalMobility *>(
                mobilityModule
            );

        auto *externalApp =
            dynamic_cast<ExternalUdpApp *>(
                applicationModule
            );

        if (externalMobility == nullptr) {
            throw cRuntimeError(
                "Module '%s' is not ExternalMobility",
                mobilityPath.c_str()
            );
        }

        if (externalApp == nullptr) {
            throw cRuntimeError(
                "Module '%s' is not ExternalUdpApp",
                appPath.c_str()
            );
        }

        mobilities.push_back(externalMobility);
        udpApps.push_back(externalApp);
    }

    const std::string edgeActorName =
        par("edgeActorName").stringValue();
    const std::string edgeAppPath =
        par("edgeAppModule").stringValue();
    const std::string edgeHostPath =
        par("edgeHostModule").stringValue();

    auto *edgeApp = dynamic_cast<ExternalUdpApp *>(
        getModuleByPath(edgeAppPath.c_str())
    );
    auto *edgeHost = getModuleByPath(edgeHostPath.c_str());

    if (edgeActorName.empty())
        throw cRuntimeError("edgeActorName cannot be empty");
    if (edgeApp == nullptr) {
        throw cRuntimeError(
            "Module '%s' was not found or is not ExternalUdpApp",
            edgeAppPath.c_str()
        );
    }
    if (edgeHost == nullptr) {
        throw cRuntimeError(
            "edge host module '%s' was not found",
            edgeHostPath.c_str()
        );
    }

    endpointApps[edgeActorName] = edgeApp;
    endpointHosts[edgeActorName] = edgeHost;

    const std::filesystem::path messageLogPath(
        par("messageLog").stringValue()
    );

    if (messageLogPath.has_parent_path())
        std::filesystem::create_directories(
            messageLogPath.parent_path()
        );

    messageLog.open(messageLogPath);

    if (!messageLog)
        throw cRuntimeError(
            "cannot open message log '%s'",
            messageLogPath.string().c_str()
        );

    messageLog
        << "event,message_id,source_actor,"
           "destination_actor,receiver_actor,"
           "generation_time,injection_time,"
           "network_receive_time,payload_size\n";

    const std::filesystem::path logPath(par("mobilityLog").stringValue());
    if (logPath.has_parent_path())
        std::filesystem::create_directories(logPath.parent_path());
    log.open(logPath);
    if (!log)
        throw cRuntimeError("cannot open mobility log '%s'", logPath.string().c_str());
    log
        << "step,frame,sim_time,ue_index,"
           "actor_id,actor_name,"
           "raw_x,raw_y,raw_z,"
           "x,y,z,vx,vy,vz,yaw\n";

    context = std::make_unique<zmq::context_t>(1);
    socket = std::make_unique<zmq::socket_t>(*context, zmq::socket_type::req);
    socket->set(zmq::sockopt::linger, 0);
    const int timeoutMs = par("timeoutMs");
    socket->set(zmq::sockopt::rcvtimeo, timeoutMs);
    socket->set(zmq::sockopt::sndtimeo, timeoutMs);
    socket->connect(par("endpoint").stringValue());

    json init = {
        {"type", "INIT"},
        {"protocol_version", 3},
        {"dt", stepSize.dbl()},
        {"duration", duration},
        {"seed", seed},
    };
    json initialState = exchange(init);
    applyState(initialState);
    injectOutgoingMessages(initialState);
    initialized = true;

    stepEvent = new cMessage("carlaStep");
    scheduleAt(simTime() + stepSize, stepEvent);
}

void CarlaCoordinator::handleMessage(cMessage *message)
{
    if (message != stepEvent)
        throw cRuntimeError("unexpected message '%s'", message->getName());

    const double requestedTime = simTime().dbl();
    json request = {
        {"type", "STEP"},
        {"sim_time", requestedTime},
        {"delivered_messages", deliveredMessages},
    };

    json response = exchange(request);

    // Only clear after a successful exchange.
    deliveredMessages = json::array();

    applyState(response);
    injectOutgoingMessages(response);

    if (simTime() + stepSize <= SimTime(duration))
        scheduleAt(simTime() + stepSize, stepEvent);
}

json CarlaCoordinator::exchange(const json& request)
{
    const std::string payload = request.dump();
    try {
        const auto sent = socket->send(zmq::buffer(payload), zmq::send_flags::none);
        if (!sent)
            throw cRuntimeError("timed out sending to the OpenCDA bridge");

        zmq::message_t reply;
        const auto received = socket->recv(reply, zmq::recv_flags::none);
        if (!received)
            throw cRuntimeError("timed out waiting for the OpenCDA bridge");

        const std::string replyText(static_cast<const char *>(reply.data()), reply.size());
        json response = json::parse(replyText);
        if (response.value("type", "") == "ERROR")
            throw cRuntimeError("OpenCDA bridge error: %s", response.value("message", "unknown").c_str());
        return response;
    }
    catch (const zmq::error_t& error) {
        throw cRuntimeError("ZeroMQ failure: %s", error.what());
    }
    catch (const json::exception& error) {
        throw cRuntimeError("invalid bridge JSON: %s", error.what());
    }
}

void CarlaCoordinator::injectOutgoingMessages(
    const json& state)
{
    if (!state.contains("outgoing_messages"))
        return;

    for (
        const auto& message :
        state.at("outgoing_messages")
    ) {
        const long messageId =
            message.at("message_id")
            .get<long>();

        const int payloadSize =
            message.at("payload_size")
            .get<int>();

        const std::string sourceActor =
            message.at("source_actor")
            .get<std::string>();

        const std::string destinationActor =
            message.at("destination_actor")
            .get<std::string>();

        const auto sourceMapping =
            endpointApps.find(sourceActor);

        const auto destinationMapping =
            endpointHosts.find(destinationActor);

        if (
            sourceMapping == endpointApps.end()
            || destinationMapping == endpointHosts.end()
        ) {
            throw cRuntimeError(
                "unknown network source '%s' or destination '%s'",
                sourceActor.c_str(),
                destinationActor.c_str()
            );
        }

        if (
            inFlightMessages.count(messageId)
            != 0
        ) {
            throw cRuntimeError(
                "duplicate message ID %ld",
                messageId
            );
        }

        ExternalUdpApp *sourceApp =
            sourceMapping->second;

        cModule *destinationModule =
            destinationMapping->second;

        const inet::L3Address destinationAddress =
            inet::L3AddressResolver()
            .addressOf(destinationModule);

        inFlightMessages[messageId] =
            message;

        messageLog
            << "INJECTED,"
            << messageId << ','
            << sourceActor << ','
            << destinationActor << ','
            << ','
            << message.at(
                "generation_time"
            ).get<double>() << ','
            << simTime().dbl() << ','
            << ','
            << payloadSize
            << '\n';

        messageLog.flush();

        sourceApp->sendExternalMessage(
            messageId,
            payloadSize,
            destinationAddress,
            par("v2vPort")
        );
    }
}

void CarlaCoordinator::applyState(
    const json& state)
{
    if (state.value("type", "") != "STATE") {
        throw cRuntimeError(
            "expected STATE response, got '%s'",
            state.value(
                "type",
                "missing"
            ).c_str()
        );
    }

    const json& actors = state.at("actors");

    if (!actors.is_array()) {
        throw cRuntimeError(
            "STATE field 'actors' is not an array"
        );
    }

    if (actors.size() != mobilities.size()) {
        throw cRuntimeError(
            "received %zu actors, but configured %zu UEs",
            actors.size(),
            mobilities.size()
        );
    }

    // Establish the name-to-UE mapping during INIT.
    if (actorToUe.empty()) {
        for (
            std::size_t i = 0;
            i < actors.size();
            ++i
        ) {
            const std::string actorName =
                actors.at(i)
                .at("name")
                .get<std::string>();

            if (actorToUe.count(actorName) != 0) {
                throw cRuntimeError(
                    "duplicate actor name '%s'",
                    actorName.c_str()
                );
            }

            actorToUe[actorName] =
                static_cast<int>(i);

            if (endpointApps.count(actorName) != 0
                    || endpointHosts.count(actorName) != 0) {
                throw cRuntimeError(
                    "actor name '%s' conflicts with a static network endpoint",
                    actorName.c_str()
                );
            }

            endpointApps[actorName] = udpApps.at(i);
            endpointHosts[actorName] =
                getParentModule()->getSubmodule(
                    "ue",
                    static_cast<int>(i)
                );

            if (endpointHosts[actorName] == nullptr) {
                throw cRuntimeError(
                    "UE module %zu for actor '%s' does not exist",
                    i,
                    actorName.c_str()
                );
            }
        }
    }

    for (const auto& actor : actors) {
        const std::string actorName =
            actor.at("name")
            .get<std::string>();

        auto mapping =
            actorToUe.find(actorName);

        if (mapping == actorToUe.end()) {
            throw cRuntimeError(
                "no UE mapping for actor '%s'",
                actorName.c_str()
            );
        }

        const int ueIndex = mapping->second;

        const double rawX =
            actor.at("x").get<double>();

        const double rawY =
            actor.at("y").get<double>();

        const double rawZ =
            actor.at("z").get<double>();

        const double x =
            rawX + xOffset;

        const double y =
            invertY
            ? yOffset - rawY
            : rawY + yOffset;

        const double z =
            std::max(
                0.0,
                rawZ + zOffset
            );

        const double vx =
            actor.at("vx").get<double>();

        const double vy =
            invertY
            ? -actor.at("vy").get<double>()
            : actor.at("vy").get<double>();

        const double vz =
            actor.at("vz").get<double>();

        mobilities.at(ueIndex)
            ->setExternalState(
                inet::Coord(x, y, z),
                inet::Coord(vx, vy, vz)
            );

        log
            << state.at("step").get<long>()
            << ','
            << state.at("frame").get<long>()
            << ','
            << state.at("sim_time").get<double>()
            << ','
            << ueIndex
            << ','
            << actor.at("id").get<long>()
            << ','
            << actorName
            << ','
            << rawX << ','
            << rawY << ','
            << rawZ << ','
            << x << ','
            << y << ','
            << z << ','
            << vx << ','
            << vy << ','
            << vz << ','
            << actor.at("yaw").get<double>()
            << '\n';
    }

    log.flush();
}

void CarlaCoordinator::recordDelivery(
    long messageId,
    const std::string& receiverActor,
    double networkReceiveTime)
{
    Enter_Method(
        "recordDelivery(messageId=%ld)",
        messageId
    );

    auto entry =
        inFlightMessages.find(messageId);

    if (entry == inFlightMessages.end()) {
        throw cRuntimeError(
            "received unknown message ID %ld",
            messageId
        );
    }

    json delivered = entry->second;

    const std::string expectedReceiver =
        delivered.at("destination_actor")
        .get<std::string>();

    if (receiverActor != expectedReceiver) {
        throw cRuntimeError(
            "message %ld expected receiver '%s', "
            "but was received by '%s'",
            messageId,
            expectedReceiver.c_str(),
            receiverActor.c_str()
        );
    }

    delivered["receiver_actor"] =
        receiverActor;

    delivered["network_receive_time"] =
        networkReceiveTime;

    deliveredMessages.push_back(delivered);

    messageLog
        << "RECEIVED,"
        << messageId << ','
        << delivered.at(
            "source_actor"
        ).get<std::string>() << ','
        << expectedReceiver << ','
        << receiverActor << ','
        << delivered.at(
            "generation_time"
        ).get<double>() << ','
        << ','
        << networkReceiveTime << ','
        << delivered.at(
            "payload_size"
        ).get<int>()
        << '\n';

    messageLog.flush();

    inFlightMessages.erase(entry);
}
void CarlaCoordinator::stopRemote() noexcept
{
    if (!initialized || remoteStopped || socket == nullptr)
        return;
    try {
        (void)exchange({{"type", "STOP"}});
    }
    catch (...) {
        // Simulation shutdown must continue even if the peer has already exited.
    }
    remoteStopped = true;
}

void CarlaCoordinator::finish()
{
    stopRemote();
    if (log.is_open())
        log.close();
    if (messageLog.is_open())
        messageLog.close();
}

CarlaCoordinator::~CarlaCoordinator()
{
    if (stepEvent != nullptr)
        cancelAndDelete(stepEvent);
    if (socket != nullptr)
        socket->close();
    socket.reset();
    context.reset();
}
