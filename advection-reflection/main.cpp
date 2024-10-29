#include <igl/writeOFF.h>
#include <thread>
#include "Gui.h"
#include "Simulator.h"
#include "FluidSim.h"

/*
 * GUI for the spring simulation. This time we need additional paramters,
 * e.g. which integrator to use for the simulation and the force applied to the
 * canonball, and we also add some more visualizations (trajectories).
 */
class FluidGui : public Gui {
public:

	FluidSim* p_fluidSim = NULL;

	FluidGui() {
		turnOffLight(); // no light for field visualization
		orthoCam(); // orthographic camera

		p_fluidSim = new FluidSim();

		setSimulation(p_fluidSim);
		setFastForward(true);
		// setParticleSystem(p_fluidSim->getParticlesData());
		start();
	}

	virtual void updateSimulationParameters() override {
		// change all parameters of the simulation to the values that are set in
		// the GUI

	}

	virtual void clearSimulation() override {

	}

	virtual void drawSimulationParameterMenu() override {

	}
};

int main(int argc, char* argv[]) {
	// create a new instance of the GUI for the spring simulation
	new FluidGui();

	return 0;
}