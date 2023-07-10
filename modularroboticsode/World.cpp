#include "World.hpp"

//COLLISION STUFF

//Used to count the number of collisions occuring in a given step
bool colliding;

//A simple class to be passed along for collision testing
class collisionData
{
	public:

		collisionData(dWorldID& world, dJointGroupID& con) : worldID(world), contacts(con)
		{
		}

		dWorldID& worldID;

		dJointGroupID& contacts;
};

//An ODE function that tests collisions between objects
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	//if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
	//{
	//	fprintf(stderr,"testing space %p %p\n", (void*)o1, (void*)o2);
	//	// colliding a space with something
	//	dSpaceCollide2(o1,o2,data,&nearCallback);
	//	// Note we do not want to test intersections within a space,
	//	// only between spaces.
	//	return;
	//}



	collisionData *dat = static_cast<collisionData*>(data);

   // Get the dynamics body for each geom

    dBodyID b1 = dGeomGetBody(o1);

    dBodyID b2 = dGeomGetBody(o2);

    // Create an array of dContact objects to hold the contact joints

    dContact contact[MAX_CONTACTS];

    // Now we set the joint properties of each contact. Going into the full details here would require a tutorial of its
       // own. I'll just say that the members of the dContact structure control the joint behaviour, such as friction,
       // velocity and bounciness. See section 7.3.7 of the ODE manual and have fun experimenting to learn more.  

    for (int i = 0; i < MAX_CONTACTS; i++)
    {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0;
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.01;
    }

    // Here we do the actual collision test by calling dCollide. It returns the number of actual contact points or zero
       // if there were none. As well as the geom IDs, max number of contacts we also pass the address of a dContactGeom
       // as the fourth parameter. dContactGeom is a substructure of a dContact object so we simply pass the address of
       // the first dContactGeom from our array of dContact objects and then pass the offset to the next dContactGeom
       // as the fifth paramater, which is the size of a dContact structure. That made sense didn't it?  

    if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact)))
    {
		colliding = true;

        // To add each contact point found to our joint group we call dJointCreateContact which is just one of the many
           // different joint types available.




        for (int i = 0; i < numc; i++)
        {
			//

			//std::cout << "World: " << dat->worldID << " Contacts: " << dat->contacts << " " << i << std::endl;

   //         // dJointCreateContact needs to know which world and joint group to work with as well as the dContact
   //            // object itself. It returns a new dJointID which we then use with dJointAttach to finally create the
   //            // temporary contact joint between the two geom bodies.

			dJointID c = dJointCreateContact(dat->worldID, dat->contacts, contact + i);
			//dJointID c = dJointCreateContact(dat->worldID, contacts2, contact + i);
            dJointAttach(c, b1, b2);
       }
    }  
}   


//WORLD CLASS

//Constructor
World::World(std::vector<std::shared_ptr<Parameters>> parameters, bool graphicsEnabled) : m_graphicsEnabled(graphicsEnabled), m_parameters(parameters), m_trialCounter(0), m_parametersCounter(0), m_delayCounter(0)
{
	std::cout << "Setting up world" << std::endl;

	//The current set of parameters is the first in the list
	m_currentParameters = m_parameters.begin();

	//Set up ODE stufff
	m_obstacleSpaceID = dSimpleSpaceCreate(0);
	m_spaceID = dSimpleSpaceCreate(0); 
	m_worldID = dWorldCreate();
	m_contacts = dJointGroupCreate(0);

	//Clear everything ready for first trial
	Reset();

	//Set up the ouput files for writing to
	std::ostringstream fileName2;

	fileName2 << "Output/Params_" << m_parametersCounter << "TimeTaken.txt";

	std::ostringstream fileName3;

	fileName3 << "Output/Params_" << m_parametersCounter << "EnergySpent.txt";

	std::ostringstream fileName4;

	fileName4 << "Output/Params_" << m_parametersCounter << "Collisions.txt";

	std::ostringstream fileName5;

	fileName5 << "Output/Params_"<< m_parametersCounter << "DistanceRemaining.txt";


	m_trialOutput.push_back(std::ofstream());
	m_trialOutput.push_back(std::ofstream());
	m_trialOutput.push_back(std::ofstream());
	m_trialOutput.push_back(std::ofstream());

	m_trialOutput[0].open(fileName2.str());

	m_trialOutput[1].open(fileName3.str());

	m_trialOutput[2].open(fileName4.str());

	m_trialOutput[3].open(fileName5.str());

	m_controlStepCounter = 0;

	std::ostringstream fileName6;

	fileName6 << "Output/Positions/" << m_parametersCounter << "_" << m_trialCounter << "positions.txt";

	m_positionOutput.open(fileName6.str());
}

void World::Update()
{
	//Delay start if required
	if (m_delayCounter < START_DELAY)
	{
		m_delayCounter ++;
		return;
	}

	//Check if trial is complete (either through victory or timeout)
	if ((m_trialLength >= MAX_TRIAL_LENGTH) || CheckVictoryConditions())
	{
		//Write trial data to files
		m_trialOutput[0] << m_trialLength << std::endl;

		m_trialOutput[1] << m_robots[0]->GetThrusterFiringCounter() << std::endl;

		m_trialOutput[2] << 100 * float(m_robots[0]->GetCollisionCounter() / float(m_trialLength / PHYSICS_STEP_TIME)) << std::endl;

		//Calculate distance to target

		m_trialOutput[3] << Vector3Df(m_robots[0]->GetPosition() - m_targetPosition).Modulus() << std::endl;

		std::cout << "Trial complete " << m_trialLength << " collisions: " << 100 * float(m_robots[0]->GetCollisionCounter() / float(m_trialLength / PHYSICS_STEP_TIME)) << std::endl;

		m_positionOutput.close();

		//Move onto next trial
		m_trialCounter ++;

		//If all trials are complete, move onto next set of parameters.
		if (m_trialCounter >= (*m_currentParameters)->m_trials)
		{
			//Record time taken
			long double duration = std::clock() - m_startTime / long double(CLOCKS_PER_SEC);

			//Create and write to parameter output file
			std::ofstream parametersOutput;

			std::ostringstream fileName;

			fileName << "Output/Params_" << m_parametersCounter << ".txt";

			parametersOutput.open(fileName.str());

			parametersOutput << duration << std::endl << (*m_currentParameters)->m_cbRtModules << std::endl << (*m_currentParameters)->m_thrusterFailureRate << std::endl;

			parametersOutput.close();




			//Reset trial counter and increment parameter counters
			m_trialCounter = 0;

			m_currentParameters ++;
			m_parametersCounter ++;




			//Close outputs files
			for (auto trialOutput = m_trialOutput.begin(); trialOutput != m_trialOutput.end(); trialOutput ++)
				trialOutput->close();

			//If all parameter sets are completed, finish
			if (m_currentParameters == m_parameters.end())
			{
				std::cout << "FINISHED" << std::endl;

				while (true) {}
			}

			//Else set up the new output files for the next set of parameters
			else
			{
				std::ostringstream fileName2;

				fileName2 << "Output/Params_" << m_parametersCounter << "TimeTaken.txt";

				std::ostringstream fileName3;

				fileName3 << "Output/Params_" << m_parametersCounter << "EnergySpent.txt";

				std::ostringstream fileName4;

				fileName4 << "Output/Params_" << m_parametersCounter << "Collisions.txt";

				std::ostringstream fileName5;

				fileName5 << "Output/Params_" << m_parametersCounter << "DistanceRemaining.txt";


				m_trialOutput[0].open(fileName2.str());

				m_trialOutput[1].open(fileName3.str());

				m_trialOutput[2].open(fileName4.str());

				m_trialOutput[3].open(fileName5.str());

				//Restart parameter clock
				m_startTime = std::clock();
			}
		}

		else
		{
			std::ostringstream fileName6;

			fileName6 << "Output/Positions/" << m_parametersCounter << "_" << m_trialCounter << "positions.txt";

			m_positionOutput.open(fileName6.str());
		}


		//Reset the world for the next trial
		Reset();
	}

	//Update the world
	Step();

	//Count trial length in simulation time
	m_trialLength += PHYSICS_STEP_TIME;
}

//The main world update function. Updates the robot
void World::Step()
{

	m_positionOutput << m_robots[0]->GetPosition().Getx() << "," << m_robots[0]->GetPosition().Gety() << "," << m_robots[0]->GetPosition().Getz() << std::endl;

	m_controlStepCounter -= PHYSICS_STEP_TIME;

	if (m_controlStepCounter <= 0)
	{
		//Update robot controller
		for (auto robotIt = m_robots.begin(); robotIt != m_robots.end(); robotIt ++)
			(*robotIt)->ControlStep(m_targetPosition);

		m_controlStepCounter = CONTROL_STEP_TIME;
	}

	//Add thruster and drag forces
	for (auto robotIt = m_robots.begin(); robotIt != m_robots.end(); robotIt ++)
	{
		(*robotIt)->ApplyDrag();
		(*robotIt)->ApplyThrusterForces();
	}


	//Handle colisions - not required for single robot with no obstacles - also calls the step function
	if ((*m_currentParameters)->m_scenario == OBSTACLES)
		HandleCollisions();

	else
		dWorldQuickStep(m_worldID, PHYSICS_STEP_TIME);

	//Draw if needed
	if (m_graphicsEnabled)
		Draw();
}

//Handles collisions - see ODE wiki
void World::HandleCollisions()
{
	colliding = false;

	collisionData* dat = new collisionData(m_worldID, m_contacts);

	//Collide robot modules with obstacles
	dSpaceCollide2(dGeomID(m_spaceID), dGeomID(m_obstacleSpaceID), dat, &nearCallback);

	if (colliding)
		m_robots.back()->IncrementCollisionCounter();

	//In the debris scenario, collide obstacles with one another
	dSpaceCollide(m_obstacleSpaceID, dat, &nearCallback);

	//For multiple robots, collide them with one another
	dSpaceCollide(m_spaceID, dat, &nearCallback);

	dWorldStep(m_worldID, PHYSICS_STEP_TIME);

	//dWorldQuickStep(m_worldID, PHYSICS_STEP_TIME);

	dJointGroupEmpty(m_contacts);

	delete dat;
}

//Reset
void World::Reset()
{
	//Destroy robots
	m_robots.clear();

	//Destroy all objects
	m_obstacles.clear();

	//Set parameters
	THRUST_FORCE = (*m_currentParameters)->m_thrusterForce;
	MODULE_SIZE = (*m_currentParameters)->m_moduleSize;
	PID_TRANS_D_TO_TRANS_P_RATIO = (*m_currentParameters)->m_PIDRatio1;
	PID_ROT_P_TO_TRANS_P_RATIO = (*m_currentParameters)->m_PIDRatio2;
	PID_ROT_D_TO_TRANS_P_RATIO = (*m_currentParameters)->m_PIDRatio3;
	PHYSICS_STEP_TIME = (*m_currentParameters)->m_physicsStepTime;
	CONTROL_STEP_TIME = (*m_currentParameters)->m_controlStepTime;
	OBSTACLE_SENSOR_RANGE = (*m_currentParameters)->m_obstacleSensorRange;
	FLUID_DENSITY = (*m_currentParameters)->m_fluidDensity;
	ROBOT_DENSITY = (*m_currentParameters)->m_robotDensity;
	
	m_controlStepCounter = 0;

	//Set new seed
	SeedRand((*m_currentParameters)->m_startingSeed + m_trialCounter);

	//Setup target position
	SetupTargetPosition();

	//Add new robot(s)
	SetupRobot();

	//Add new objects
	SetupObstacles();

	//Reset trial length
	m_trialLength = 0;
}

void World::SetupRobot()
{
	std::vector<Vector3Df> scaledModuleMap;

	std::vector<Vector3Di> unscaledModuleMap;
	
	//Two module maps are created, one with a unit distance between adjacent cells and one with a module width distance between adjacent cells

	//Default map, just in case nothing else is created

	scaledModuleMap.push_back(Vector3Df(0, 0, 0));

	unscaledModuleMap.push_back(Vector3Di(0, 0, 0));

	//Create a 'convex' robot - that is, a robot for which no straight line exists that is aligned with one of the cardinal axes and can pass through an external of the robot more than twice (no 'holes')
	if ((*m_currentParameters)->m_robotType == CONVEX)
	{
		int numModules = (*m_currentParameters)->m_cbRtModules * (*m_currentParameters)->m_cbRtModules * (*m_currentParameters)->m_cbRtModules;



		for (int i = 0; i != numModules - 1; i ++)
		{
			while (true)
			{
				//Select random module from exisiting modules

				int moduleToAttach = RandFloat() * scaledModuleMap.size();

				int faceToAttach = RandFloat() * 6;

				//New module position
				//Vector3Df newPosition = scaledModuleMap[moduleToAttach] + MODULE_SIZE * directions[faceToAttach];

				Vector3Di delta(int(directions[faceToAttach].Getx()), int(directions[faceToAttach].Gety()), int(directions[faceToAttach].Getz()));

				Vector3Di newPosition = unscaledModuleMap[moduleToAttach] + delta;

				//Check that another module isn't attached here already
				float canAttach = true;

				for (auto modIt = unscaledModuleMap.begin(); modIt != unscaledModuleMap.end(); modIt ++)
				{
					if (*modIt == newPosition)
					{
						canAttach = false;

						break;
					}
				}

				//Add the new module to a temporary map before confirming it will be added permenantly
				std::vector<Vector3Di> moduleMapPlusNewModule = unscaledModuleMap;

				moduleMapPlusNewModule.push_back(newPosition);

				//Check that orthogonal convexity is preserved

				//After adding a new module, determine the 6 extremum modules

				int extrema[6] = {0, 0, 0, 0, 0, 0};

				for (auto modIt = moduleMapPlusNewModule.begin(); modIt != moduleMapPlusNewModule.end(); modIt ++)
				{
					if ((modIt->Getx() > extrema[0]))
						extrema[0] = modIt->Getx();

					if ((modIt->Getx() < extrema[1]))
						extrema[1] = modIt->Getx();

					if ((modIt->Gety() > extrema[2]))
						extrema[2] = modIt->Gety();

					if ((modIt->Gety() < extrema[3]))
						extrema[3] = modIt->Gety();

					if ((modIt->Getz() > extrema[4]))
						extrema[4] = modIt->Getz();

					if ((modIt->Getz() < extrema[5]))
						extrema[5] = modIt->Getz();
				}

				//Check in all 6 directions from all modules to the extremum module that there is no hole

				//Iterate across all modules in the temporary map
				for (auto modIt = moduleMapPlusNewModule.begin(); modIt != moduleMapPlusNewModule.end(); modIt ++)
				{
					//Iterate across each of the cartesian axes (in both directions)
					for (int direction = 0; direction != 6; direction ++)
					{
						//Starting checking at this module position
						Vector3Di positionToCheck = *modIt;

						bool hasHole = false;

						//Iterate along each axes until the extrema in this direction is reached
						while (positionToCheck[direction / 2] != extrema[direction])
						{
							//Move to the next grid position 
							positionToCheck = positionToCheck + Vector3Di(directions[direction].Getx(), directions[direction].Gety(), directions[direction].Getz());

							//Check if a module is contained within the map at this position
							bool mapContainsModule = false;

							for (auto mod2It = moduleMapPlusNewModule.begin(); mod2It != moduleMapPlusNewModule.end(); mod2It ++)
							{
								if (*mod2It == positionToCheck)
								{
									mapContainsModule = true;

									break;
								}
							}

							//If a second hole is found, the robot is not convex and so this module cannot be attached
							if (hasHole && mapContainsModule)
								canAttach = false;

							//If no module is present here, there is a hole
							if (!mapContainsModule)
								hasHole = true;
						}
					}
				}
		

				//If there is no reason that the module cannot be attached in this position, then add it to both module maps
				if (canAttach)
				{
					unscaledModuleMap.push_back(newPosition);
					scaledModuleMap.push_back(Vector3Df(newPosition.Getx() * MODULE_SIZE, newPosition.Gety() * MODULE_SIZE, newPosition.Getz() * MODULE_SIZE));

					break;
				}
			}
		}
	}

	//if ((*m_currentParameters)->m_robotType == CONVEX)
	//{
	//	int numModules = (*m_currentParameters)->m_cbRtModules * (*m_currentParameters)->m_cbRtModules * (*m_currentParameters)->m_cbRtModules;

	//	for (int i = 0; i != numModules - 1; i ++)
	//	{
	//		while (true)
	//		{

	//			//Select random module from exisiting modules

	//			int moduleToAttach = RandFloat() * scaledModuleMap.size();

	//			int faceToAttach = RandFloat() * 6;

	//			//New module position
	//			//Vector3Df newPosition = scaledModuleMap[moduleToAttach] + MODULE_SIZE * directions[faceToAttach];

	//			Vector3Di delta(int(directions[faceToAttach].Getx()), int(directions[faceToAttach].Gety()), int(directions[faceToAttach].Getz()));

	//			Vector3Di newPosition = unscaledModuleMap[moduleToAttach] + delta;

	//			//Check that another module isn't attached here already
	//			float canAttach = true;

	//			for (auto modIt = unscaledModuleMap.begin(); modIt != unscaledModuleMap.end(); modIt ++)
	//			{
	//				if (*modIt == newPosition)
	//				{
	//					canAttach = false;

	//					break;
	//				}
	//			}

	//			std::vector<Vector3Di> moduleMapPlusNewModule = unscaledModuleMap;

	//			moduleMapPlusNewModule.push_back(newPosition);

	//			//Check that orthogonal convexity is preserved

	//			//Find the 6 extremum modules

	//			int extrema[6] = {0, 0, 0, 0, 0, 0};

	//			for (auto modIt = moduleMapPlusNewModule.begin(); modIt != moduleMapPlusNewModule.end(); modIt ++)
	//			{
	//				if ((modIt->Getx() > extrema[0]))
	//					extrema[0] = modIt->Getx();

	//				if ((modIt->Getx() < extrema[1]))
	//					extrema[1] = modIt->Getx();

	//				if ((modIt->Gety() > extrema[2]))
	//					extrema[2] = modIt->Gety();

	//				if ((modIt->Gety() < extrema[3]))
	//					extrema[3] = modIt->Gety();

	//				if ((modIt->Getz() > extrema[4]))
	//					extrema[4] = modIt->Getz();

	//				if ((modIt->Getz() < extrema[5]))
	//					extrema[5] = modIt->Getz();
	//			}

	//			//Check in all 6 directions from all modules to the extremum module that there is no hole

	//			//std::cout << "Position: "  << newPosition.Getx() << " " << newPosition.Gety() << " " << newPosition.Getz() << std::endl;




	//			for (auto modIt = moduleMapPlusNewModule.begin(); modIt != moduleMapPlusNewModule.end(); modIt ++)
	//			{
	//				for (int direction = 0; direction != 6; direction ++)
	//				{
	//					Vector3Di positionToCheck = *modIt;

	//					bool hasHole = false;

	//					//std::cout << "Direction: " << directions[direction].Getx() << " " << directions[direction].Gety() << " " << directions[direction].Getz() << std::endl;

	//					//std::cout << "PosToCheck: " << positionToCheck[direction / 2] << "       extrema: " << extrema[direction] << std::endl;

	//					while (positionToCheck[direction / 2] != extrema[direction])
	//					{
	//						positionToCheck = positionToCheck + Vector3Di(directions[direction].Getx(), directions[direction].Gety(), directions[direction].Getz());

	//						//std::cout << "Check: " << positionToCheck.Getx() << " " << positionToCheck.Gety() << " " << positionToCheck.Getz() << std::endl;

	//						bool mapContainsModule = false;

	//						for (auto mod2It = moduleMapPlusNewModule.begin(); mod2It != moduleMapPlusNewModule.end(); mod2It ++)
	//						{
	//							if (*mod2It == positionToCheck)
	//							{
	//								mapContainsModule = true;

	//								break;
	//							}
	//						}

	//						if (hasHole && mapContainsModule)
	//							canAttach = false;

	//						if (!mapContainsModule)
	//							hasHole = true;
	//					}
	//				}
	//			}
	//	


	//			if (canAttach)
	//			{
	//				unscaledModuleMap.push_back(newPosition);
	//				scaledModuleMap.push_back(Vector3Df(newPosition.Getx() * MODULE_SIZE, newPosition.Gety() * MODULE_SIZE, newPosition.Getz() * MODULE_SIZE));

	//				break;
	//			}
	//		}
	//	}
	//}



	//Create a random robot. Each module must form an unbroken set of face-to-face connections with every other module. Otherwise, there are no constraints on the topology
	else if ((*m_currentParameters)->m_robotType == RANDOM)
	{
		int numModules = (*m_currentParameters)->m_cbRtModules * (*m_currentParameters)->m_cbRtModules * (*m_currentParameters)->m_cbRtModules;

		for (int i = 0; i != numModules - 1; i ++)
		{
			while (true)
			{

				//Select random module from exisiting modules

				int moduleToAttach = RandFloat() * scaledModuleMap.size();

				int faceToAttach = RandFloat() * 6;

				Vector3Di delta(int(directions[faceToAttach].Getx()), int(directions[faceToAttach].Gety()), int(directions[faceToAttach].Getz()));

				Vector3Di newPosition = unscaledModuleMap[moduleToAttach] + delta;

				//Check that another module isn't attached here already
				float canAttach = true;

				for (auto modIt = unscaledModuleMap.begin(); modIt != unscaledModuleMap.end(); modIt ++)
				{
					if (*modIt == newPosition)
					{
						canAttach = false;

						break;
					}
				}

				//If there is no reason that the module cannot be attached in this position, then add it to both module maps
				if (canAttach)
				{
					unscaledModuleMap.push_back(newPosition);
					scaledModuleMap.push_back(Vector3Df(newPosition.Getx() * MODULE_SIZE, newPosition.Gety() * MODULE_SIZE, newPosition.Getz() * MODULE_SIZE));

					break;
				}
			}
		}
	}

	//Create a cubic robot
	else if ((*m_currentParameters)->m_robotType == CUBIC)
	{
		scaledModuleMap.clear();
		unscaledModuleMap.clear();

		for (int i = 0; i != (*m_currentParameters)->m_cbRtModules; i ++)
		{
			for (int j = 0; j != (*m_currentParameters)->m_cbRtModules; j ++)
			{
				for (int k = 0; k != (*m_currentParameters)->m_cbRtModules; k ++)
				{
					unscaledModuleMap.push_back(Vector3Di(i, j , k));
					scaledModuleMap.push_back(Vector3Df(i * MODULE_SIZE, j * MODULE_SIZE , k * MODULE_SIZE));
				}
			}
		}
	}

	//Reseed so that seed is always consistent for both random robot generation and generation of initial conditions
	SeedRand((*m_currentParameters)->m_startingSeed + m_trialCounter);

	//Add the robot

	MatrixNMf orientation = GenerateStartingOrientation();

	//Shift the positions of the scaled module map so that the center of mass lies at (0, 0, 0)
	std::vector<Vector3Df> convertedModuleMap = ConvertMap(scaledModuleMap);

	m_robots.push_back(std::shared_ptr<Robot>(new Robot(m_worldID, m_spaceID, m_obstacleSpaceID, convertedModuleMap, unscaledModuleMap, STARTING_POSITION, orientation, *m_currentParameters)));


}

//Set up the obstacles within the environment depending on the scenario patameter
void World::SetupObstacles()
{
	//Set up obstacles depending on the scenario type

	if ((*m_currentParameters)->m_scenario == OBSTACLES)
		GenerateSlalomObstacles();
}

//Generate the robot target position, with a distance from the starting point determined by the target distance parameter and half the robot size (so that the distance to target point is approximately independent of robot size)
void World::SetupTargetPosition()
{
	//Target point is starting position + target distance + 1/2 * robot side length
	m_targetPosition = STARTING_POSITION + Vector3Df((*m_currentParameters)->m_targetDistance + 0.5 * float((*m_currentParameters)->m_cbRtModules) * (*m_currentParameters)->m_moduleSize, 0, 0);
}

//Iterates through a module map, calculates the center of mass position, and creates a new map with the space spacing and a center of mass of (0, 0, 0)
std::vector<Vector3Df> World::ConvertMap(std::vector<Vector3Df> map)
{
	//Calculate center of mass

	Vector3Df CoM(0, 0, 0);

	for (auto mod = map.begin(); mod != map.end(); mod ++)
	{
		CoM.Setx(CoM.Getx() + mod->Getx());
		CoM.Sety(CoM.Gety() + mod->Gety());
		CoM.Setz(CoM.Getz() + mod->Getz());
	}

	CoM.Setx(CoM.Getx() / map.size());
	CoM.Sety(CoM.Gety() / map.size());
	CoM.Setz(CoM.Getz() / map.size());

	//center of mass should be 0, 0, 0 and everything else scaled around it

	std::vector<Vector3Df> newMap;

	for (auto mod = map.begin(); mod != map.end(); mod ++)
	{
		newMap.push_back(*mod - CoM);
	}

	return newMap;
}

//Draw all of the objects within the environment
void World::Draw()
{
	//Draw target sphere
	dsSetColor (0, 0.6, 0);

	float R[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
	dVector3 T;
	T[0] = m_targetPosition[0];
	T[1] = m_targetPosition[1];
	T[2] = m_targetPosition[2];

	dsDrawSphere(T, R, (*m_currentParameters)->m_moduleSize);


	//Draw starting sphere
	dsSetColor (0.6, 0, 0);

	T[0] = STARTING_POSITION.Getx();
	T[1] = STARTING_POSITION.Gety();
	T[2] = STARTING_POSITION.Getz();

	dsDrawSphere(T, R, (*m_currentParameters)->m_moduleSize);

	//Draw obstacles
	for (auto obIt = m_obstacles.begin(); obIt != m_obstacles.end(); obIt ++)
	{
		(*obIt)->Draw();
	}

	//Draw robots
	for (auto robotIt = m_robots.begin(); robotIt != m_robots.end(); robotIt ++)
	{
		(*robotIt)->Draw();
	}
}

//Check if the robot has reached the target position - check if the target point is within the axis-aligned bounding box (AABB) of the robot
bool World::CheckVictoryConditions()
{
	//target point in robot body coordsinates

	dVector3 robotTarget;

	dBodyGetPosRelPoint(m_robots.back()->GetBodyID(), m_targetPosition.Getx(), m_targetPosition.Gety(), m_targetPosition.Getz(), robotTarget);

	std::vector<float> AABB = m_robots.back()->GetAABB();

	if (robotTarget[0] > AABB[0] && robotTarget[0] < AABB[1] && robotTarget[1] > AABB[2] && robotTarget[1] < AABB[3] && robotTarget[2] > AABB[4] && robotTarget[2] < AABB[5])
	{
		return true;
	}

	/*dReal pos[3];
				
	pos[0] = dBodyGetPosition(m_robots.back()->GetBodyID())[0];
	pos[1] = dBodyGetPosition(m_robots.back()->GetBodyID())[1];
	pos[2] = dBodyGetPosition(m_robots.back()->GetBodyID())[2];

	dReal delta[3];

	delta[0] = pos[0] - m_targetPosition.Getx();
	delta[1] = pos[1] - m_targetPosition.Gety();
	delta[2] = pos[2] - m_targetPosition.Getz();

	if (sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]) < MODULE_SIZE * float((*m_currentParameters)->m_cbRtModules))
	{
		return true;
	}
	
	return false;*/

	return false;
}

//Add a new obstacle to the environment that the robot can collide with
void World::AddNewObstacle(Vector3Df size, Vector3Df position)
{
	//Add an obstacle. Depending on the scenario, the obstacles should be kinematic or not

	/*if ((*m_currentParameters)->m_scenario == DEBRIS)
		m_obstacles.push_back(std::shared_ptr<Obstacle>(new Obstacle(size, position, m_worldID, m_obstacleSpaceID, false)));

	else if ((*m_currentParameters)->m_scenario == SLALOM)*/
		m_obstacles.push_back(std::shared_ptr<Obstacle>(new Obstacle(size, position, m_worldID, m_obstacleSpaceID, true)));
}

//Generate a random debris field
void World::GenerateRandomDebris()
{
	//Each piece of debris needs random position, random rotation and random size

	for (int i = 0; i != DEBRIS_NUMBER; i ++)
	{
		Vector3Df size;

		size.Setx(DEBRIS_MIN_SIZE + RandFloat() * (DEBRIS_MAX_SIZE - DEBRIS_MIN_SIZE));
		size.Sety(DEBRIS_MIN_SIZE + RandFloat() * (DEBRIS_MAX_SIZE - DEBRIS_MIN_SIZE));
		size.Setz(DEBRIS_MIN_SIZE + RandFloat() * (DEBRIS_MAX_SIZE - DEBRIS_MIN_SIZE));

		Vector3Df position;

		position.Setx(RandFloat() * -((*m_currentParameters)->m_targetDistance + 20) + 10);
		position.Sety(RandFloat() * 30 - 15);
		position.Setz(RandFloat() * 30 + 115);

		/*position.Setx(2 + RandFloat() * (m_targetPosition[0] - 4));
		position.Sety(2 + RandFloat() * (m_targetPosition[1] - 4));
		position.Setz(2 + RandFloat() * (m_targetPosition[2] - 4));*/

		AddNewObstacle(size, position);
	}
}

//Generate two obstacles for the slalom
void World::GenerateSlalomObstacles()
{
	AddNewObstacle(Vector3Df(0.5, 0.5, 0.5), STARTING_POSITION + Vector3Df(1 + RandFloat() * 0.5, -0.25 + RandFloat() * 0.5, -0.25 + RandFloat() * 0.5));

	AddNewObstacle(Vector3Df(0.5, 0.5, 0.5), STARTING_POSITION + Vector3Df(4 - RandFloat() * 0.5, -0.25 + RandFloat() * 0.5, -0.25 + RandFloat() * 0.5));
}

//Generate the starting oreienation of the robot depending on the run parameters - either random or fixed
MatrixNMf World::GenerateStartingOrientation()
{

	if((*m_currentParameters)->m_randomOrientation)
	{
		//Generate uniformly random rotation matrix
		//http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.53.1357

		float theta = RandFloat() * 2 * PI;
		float phi = RandFloat() * 2 * PI;
		float z = RandFloat();
		MatrixNMf V(3, 1);

		//Identity
		MatrixNMf I(3, 3);

		//Rotation
		MatrixNMf R(3, 3);

		R.SetElement(0, 0, cos(theta));
		R.SetElement(1, 1, cos(theta));
		R.SetElement(1, 0, -sin(theta));
		R.SetElement(0, 1, sin(theta));

		V.SetElement(0, 0, cos(phi) * sqrt(z));
		V.SetElement(1, 0, sin(phi) * sqrt(z));
		V.SetElement(2, 0, sqrt(1 - z));

		MatrixNMf rot = ( 2 * V.MatrixMultiply(V.GetTranspose()) - I ).MatrixMultiply(R);


		return rot;
	}

	else
	{
		//Identity
		MatrixNMf I(3, 3);

		//I.SetElement(0, 0, 1);
		//I.SetElement(1, 1, 1);
		//I.SetElement(2, 2, 1);

		//Vertex on

		/*float theta = PI / float(4);
		float psi = atan(float(1) / float(sqrt(float(2))));
		float phi = 0;

		I.SetElement(0, 0, cos(theta) * cos(psi));
		I.SetElement(0, 1, -sin(psi));
		I.SetElement(0, 2, sin(theta) * cos(psi));

		I.SetElement(1, 0, cos(theta) * sin(psi));
		I.SetElement(1, 1, cos(psi));
		I.SetElement(1, 2, sin(theta) * sin(psi));

		I.SetElement(2, 0, -sin(theta));
		I.SetElement(2, 1, 0);
		I.SetElement(2, 2, cos(theta));*/


		//Edge on

		float theta = 0;
		float psi = PI / float(4);
		float phi = 0;

		I.SetElement(0, 0, cos(theta) * cos(psi));
		I.SetElement(0, 1, -sin(psi));
		I.SetElement(0, 2, sin(theta) * cos(psi));

		I.SetElement(1, 0, cos(theta) * sin(psi));
		I.SetElement(1, 1, cos(psi));
		I.SetElement(1, 2, sin(theta) * sin(psi));

		I.SetElement(2, 0, -sin(theta));
		I.SetElement(2, 1, 0);
		I.SetElement(2, 2, cos(theta));



		/*const dMatrix3 dRotation =
		{
			cos(theta) * cos(psi), -sin(psi), sin(theta) * cos(psi), 0,
			cos(theta) * sin(psi), cos(psi), sin(theta) * sin(psi), 0,
			-sin(theta), 0, cos(theta), 0
		};*/





		/*I.SetElement(1, 1, cos(PI * 0.75));
		I.SetElement(2, 2, cos(PI * 0.75));
		I.SetElement(2, 1, sin(PI * 0.75));
		I.SetElement(1, 2, -sin(PI * 0.75));*/

		/*I.SetElement(0, 0, cos(PI * 1.2));
		I.SetElement(2, 2, cos(PI * 1.2));
		I.SetElement(2, 0, sin(PI * 1.2));
		I.SetElement(0, 2, -sin(PI * 1.2));
		I.SetElement(1, 0, 0);
		I.SetElement(0, 1, 0);
		I.SetElement(1, 1, 0);
		I.SetElement(2, 1, 0);
		I.SetElement(1, 2, 0);
		I.SetElement(2, 2, 0);*/

		/*dMatrix3 dRotation =
		{
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0
		};*/

		//Backwards

		/*dMatrix3 dRotation =
		{
			1, 0, 0, 0,
			0, cos(PI * 0.75), -sin(PI * 0.75), 0,
			0, sin(PI * 0.75), cos(PI * 0.75), 0
		};*/

		//Vertex on

		/*float theta = PI / float(4);
		float psi = atan(float(1) / float(sqrt(float(2))));
		float phi = 0;

		const dMatrix3 dRotation =
		{
			cos(theta) * cos(psi), -sin(psi), sin(theta) * cos(psi), 0,
			cos(theta) * sin(psi), cos(psi), sin(theta) * sin(psi), 0,
			-sin(theta), 0, cos(theta), 0
		};*/

		/*const dMatrix3 dRotation =
		{
			0.25, -sqrt(float(3)) / float(4) + float(3) / float(8), float(3) / float(4) + sqrt(float(3)) / float(8), 0,
			sqrt(float(3)) / float(4), float(1) / float(4) + 3 * sqrt(float(3)) / float(8), -sqrt(float(3)) / float(4) + float(3) / float(8), 0,
			-sqrt(float(3)) / float(2), sqrt(float(3)) / float(4), 0.25, 0
		};*/


		/*float temp1 = float(1) / float(2 * sqrt(float(2))) - 0.5;

		float temp2 = float(1) / float(2 * sqrt(float(2))) + 0.5;

		const dMatrix3 dRotation =
		{
			0.5, temp1, temp2, 0,
			0.5, temp2, temp1, 0,
			-1/sqrt(float(2)), 0.5, 0.5, 0
		};*/

		/*const dMatrix3 dRotation =
		{
			0.5, -1/sqrt(float(2)), 0.5, 0,
			0.5, 1/sqrt(float(2)), 0.5, 0,
			-1/sqrt(float(2)), 0, 1/sqrt(float(2)), 0
		};*/


		//Edge on

		/*const dMatrix3 dRotation =
		{
			cos(float(45) * PI / float(180)), sin(float(45) * PI / float(180)), 0,  0,
			-sin(float(45) * PI / float(180)), cos(float(45) * PI / float(180)), 0, 0,
			0, 0, 1, 0
		};*/

		return I;
	}
}