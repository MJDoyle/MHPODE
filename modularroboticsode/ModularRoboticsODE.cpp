#include "World.hpp"

//Simulation world - all robots, targets and objects are contained within this. It runs the main update loop
std::shared_ptr<World> world;

//Default ODE setup function. Sets camera position and orientation
static void start() 
{
	static float xyz[3] = {2.5f, 0.0f, 135.0000f};	//Position
	static float hpr[3] = {90.0f, 270.000f, 0.0000f};	//Orientation

	dsSetViewpoint (xyz,hpr);
}

//Default ODE input function. Not used
static void command (int cmd)
{
}

//Default ODE update function when graphics are enabled. Calls the world update function
static void simLoop(int pause)
{
	world->Update();
}

//Main function
int main (int argc, char **argv)
{
	bool graphicsEnabled = true;	//Run with or without graphical output

	std::cout << "Starting" << std::endl;

	std::vector<std::shared_ptr<Parameters>> parameters;	//List of run parameters - number of trials, robot type, sensors used, etc.



	parameters.push_back(std::shared_ptr<Parameters>(new Parameters()));

	parameters.back()->m_scenario = OBSTACLES;	
	parameters.back()->m_controller = DECENTRALIZED_BASIC;
	parameters.back()->m_trials = 100;
	parameters.back()->m_robotType = RANDOM;
	parameters.back()->m_cbRtModules = 5;
	parameters.back()->m_randomOrientation = true;
	parameters.back()->m_thrusterForce = 0.0064;
	parameters.back()->m_moduleSize = 0.08;
	parameters.back()->m_targetDistance = 5;
	parameters.back()->m_PIDRatio1 = 0.1 * pow(float(5), 2);
	parameters.back()->m_PIDRatio2 = 0.1 * pow(float(5), 2);
	parameters.back()->m_PIDRatio3 = 0.1 * pow(float(5), 3);
	parameters.back()->m_thrusterMisalignment = 0;
	parameters.back()->m_thrusterFailureRate = 0;
	parameters.back()->m_sensorFailureRate = 0;
	parameters.back()->m_sensorNoise = 0;
	parameters.back()->m_startingSeed = 1;
	parameters.back()->m_physicsStepTime = 0.1;
	parameters.back()->m_controlStepTime = 0.1;
	parameters.back()->m_obstacleSensorRange = 0.8;
	parameters.back()->m_fluidDensity = 1000;
	parameters.back()->m_robotDensity = 1000;
	parameters.back()->m_fluidViscosity = 1;
	parameters.back()->m_dragType = QUADRATIC;	

	world = std::shared_ptr<World>(new World(parameters, graphicsEnabled));		//Create the world

	dInitODE();	//Initialize ODE;

	std::cout << "Finished setting up" << std::endl;

	//If graphics are enabled, intialize drawstuff functions and run simulation loop
	if (graphicsEnabled)
	{
		dsFunctions fn;
		fn.version = DS_VERSION;
		fn.start = &start;
		fn.step = &simLoop;
		fn.command = &command;
		fn.stop = 0;
		fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		dsSimulationLoop (argc, argv, 600, 600, &fn);	//Run the simulation loop which automatically calls the simLoop function and draws the output
	}

	//If not, just run the world update loop directly
	else
	{
		while (true)
			world->Update();
	}


	dCloseODE();	//Close ODE
	return 0;
}



