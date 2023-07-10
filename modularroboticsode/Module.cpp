#include "Module.hpp"

//Constructor
Module::Module(Vector3Df position, dMass mass, dSpaceID& space, dBodyID body, int pressureID, std::shared_ptr<Parameters> parameters)
{
	m_parameters = parameters;

	m_position = position;

	m_robotBodyID = body;

	//Create a geom if required by the scenario (allows collisions)
	if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{
		m_geom = dCreateBox(space, MODULE_SIZE, MODULE_SIZE, MODULE_SIZE);

		dGeomSetBody(m_geom, body);

		dGeomSetOffsetPosition(m_geom, m_position.Getx() - mass.c[0], m_position.Gety() - mass.c[1], m_position.Getz() - mass.c[2]);
	}
}

//Deconstructor
Module::~Module()
{
	//Destroy the geom if it was created
	if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{
		dGeomDestroy(m_geom);
	}
}

//Draw the module
void Module::Draw()
{
	//Each face is drawn individually
	for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		(*faceIt)->Draw();
	}
}


//Control step
void Module::Step(Vector3Df targetPosition)
{
	//Iterate through each face
	for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		//Only external faces contribute
		if ((*faceIt)->GetExternal())
		{
			//If using the decentralized object avoidance controller, fire the thruster if target sensor occluded (true) or obstacle detected (true)
			if (m_parameters->m_controller == DECENTRALIZED_AVOID)
			{
				if ((*faceIt)->GetTargetSensorInput(targetPosition) || (*faceIt)->GetObstacleSensorInput())
					(*faceIt)->SetThrusterFiring(true);

				else
					(*faceIt)->SetThrusterFiring(false); 
			}

			//If using the basi decentralized controller, fire the thruster if target sensor occluded (true) 
			else
			{
				if ((*faceIt)->GetTargetSensorInput(targetPosition))
					(*faceIt)->SetThrusterFiring(true);

				else
					(*faceIt)->SetThrusterFiring(false);
			}
		}
	}
}

//Sum and return the number of thrusters currently firing
int Module::GetNumFiringThrusters()
{
	int numFiringThrusters = 0;

	for (auto face = m_faces.begin(); face != m_faces.end(); face ++)
	{
		if ((*face)->GetThrusterFiring())
			numFiringThrusters ++;
	}

	return numFiringThrusters;
}