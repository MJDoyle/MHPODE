#include "Module.hpp"

Module::Module(Vector3Df position, dMass mass, dSpaceID& space, dBodyID body, int pressureID, std::shared_ptr<Parameters> parameters)
{
	m_parameters = parameters;

	m_position = position;

	m_robotBodyID = body;

	m_CoMrelativePosition = Vector3Df(m_position.Getx() - mass.c[0], m_position.Gety() - mass.c[1], m_position.Getz() - mass.c[2]);

	//Create a geom if required
	if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{
		m_geom = dCreateBox(space, MODULE_SIZE, MODULE_SIZE, MODULE_SIZE);

		dGeomSetBody(m_geom, body);

		dGeomSetOffsetPosition(m_geom, m_position.Getx() - mass.c[0], m_position.Gety() - mass.c[1], m_position.Getz() - mass.c[2]);

		std::cout << mass.c[0] << " " << mass.c[1] << " " << mass.c[2] << std::endl;
	}

}

//Deconstructor
Module::~Module()
{
	if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{
		dGeomDestroy(m_geom);
	}
}

void Module::Draw()
{
	//dsSetColor (0.8, 0, 0);

	//const dVector3 S = {MODULE_SIZE - 0.02, MODULE_SIZE - 0.02, MODULE_SIZE - 0.02};

	//dsDrawBox (dGeomGetPosition(m_geom), dGeomGetRotation(m_geom), S);





	////Draw geom position

	//const dReal* pLocalRotMat = dGeomGetRotation (m_geom);

	//dReal aabb[6];

	//dGeomGetAABB(m_geom, aabb);

	//dVector3 S = {aabb[1] - aabb[0], aabb[3] - aabb[2], aabb[5] - aabb[4]};

	//dsSetColorAlpha (0.8, 0, 0, 0.5);

	//float R[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
	//dVector3 T;
	//T[0] = dGeomGetPosition(m_geom)[0];
	//T[1] = dGeomGetPosition(m_geom)[1];
	//T[2] = dGeomGetPosition(m_geom)[2];

	////dsDrawSphere(T, pLocalRotMat, 0.2);

	////dsDrawBox(T, pLocalRotMat, S); 


	for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		(*faceIt)->Draw();
	}
}

void Module::Step(Vector3Df targetPosition)
{
	for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		if ((*faceIt)->GetExternal())
		{
			if (m_parameters->m_controller == DECENTRALIZED_AVOID)
			{
				if ((*faceIt)->GetTargetSensorInput(targetPosition) || (*faceIt)->GetObstacleSensorInput())
					(*faceIt)->SetThrusterFiring(true);

				else
					(*faceIt)->SetThrusterFiring(false); 
			}

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


std::shared_ptr<Face> Module::GetOppositeFace(Vector3Df unNormal)
{
	for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		if ((*faceIt)->GetUnNormal() == -1 * unNormal)
		{
			return (*faceIt);
		}
	}
}

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