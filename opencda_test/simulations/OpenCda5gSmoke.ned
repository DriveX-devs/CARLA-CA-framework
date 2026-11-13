package opencda_test.simulations;

import simu5g.simulations.NR.networks.SingleCell_Standalone;
import opencda_test.CarlaCoordinator;
network OpenCda5gSmoke extends SingleCell_Standalone
{
    parameters:
        //numUe = 2;
        numBgCells = 0;
        @display("bgb=1000,1000");
    submodules:
        coordinator: CarlaCoordinator {
            @display("p=80,80");
        }
}
