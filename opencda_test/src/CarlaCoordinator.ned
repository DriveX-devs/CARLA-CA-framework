package opencda_test;

simple CarlaCoordinator
{
    parameters:
        string endpoint =
            default("tcp://127.0.0.1:5555");

        double step @unit(s) =
            default(0.05s);

        double duration @unit(s) =
            default(20s);

        int seed = default(11);
        int timeoutMs = default(120000);

        int numActors = default(2);
        int v2vPort = default(4000);

        // Static application endpoint used by the existing CARLA-CA V2X
        // layer.  It has no CARLA mobility actor and runs on the INET server.
        string edgeActorName = default("BS");
        string edgeAppModule = default("^.server.app[0]");
        string edgeHostModule = default("^.server");

        string mobilityLog =
            default(
                "results/omnet_mobility.csv"
            );

        string messageLog =
            default(
                "results/omnet_messages.csv"
            );

        double xOffset @unit(m) =
            default(500m);

        double yOffset @unit(m) =
            default(500m);

        double zOffset @unit(m) =
            default(0m);

        bool invertY = default(true);

        @class(CarlaCoordinator);
}
