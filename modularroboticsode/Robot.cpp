#include "Robot.hpp"

Robot::Robot(dWorldID& worldID, dSpaceID& spaceID, dSpaceID& obstacleSpaceID, std::vector<Vector3Df> modules, std::vector<Vector3Di> unscaledModules, Vector3Df startingPosition, MatrixNMf startingOrientation, std::shared_ptr<Parameters> parameters) : m_thrusterFiringCounter(0)
{
	//Parameters
	m_parameters = parameters;

	//Set up body
	m_bodyID = dBodyCreate(worldID);

	m_robotSpaceID = spaceID;

	m_numExternalThrusters = 0;

	m_collisionCounter = 0;

	//Set up mass
	dMass totalMass;

	if (m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{
		dMass geomMass;

		//Go through modules and properly add each ones mass to the total mass
		for (std::vector<Vector3Df>::iterator modulesIt = modules.begin(); modulesIt != modules.end(); modulesIt++)
		{
			dMassSetZero (&geomMass);

			dMassSetBox(&geomMass, ROBOT_DENSITY, MODULE_SIZE, MODULE_SIZE, MODULE_SIZE);

			dMassTranslate(&geomMass, modulesIt->Getx(), modulesIt->Gety(), modulesIt->Getz());

			dMassAdd(&totalMass, &geomMass);
		}

		std::cout << totalMass.c[0] << " " << totalMass.c[1] << " " << totalMass.c[2] << std::endl;
	
		//dMassTranslate (&totalMass, -totalMass.c[0], -totalMass.c[1], -totalMass.c[2]); 

		std::cout << totalMass.c[0] << " " << totalMass.c[1] << " " << totalMass.c[2] << std::endl;
	}

	//For a cubic robot, can just make a single mass
	else
	{
		dMassSetBox(&totalMass, ROBOT_DENSITY, MODULE_SIZE * m_parameters->m_cbRtModules, MODULE_SIZE * m_parameters->m_cbRtModules, MODULE_SIZE * m_parameters->m_cbRtModules);
	}


	//std::cout << "MOUDLES NO: " << modules.size() << std::endl;

	//Add modules
	for (int i = 0; i != modules.size(); i ++)
	{
		m_modules.push_back(std::shared_ptr<Module>(new Module(modules[i], totalMass, spaceID, m_bodyID, i, parameters)));

		m_moduleMap[unscaledModules[i]] = m_modules.back();

		//std::cout << i << " " << unscaledModules[i].Getx() << " " << unscaledModules[i].Gety() << " " << unscaledModules[i].Getz() << std::endl;
	}

	//std::cout << "MAP SIZE: " << m_moduleMap.size() << std::endl;

	/*for (std::vector<Vector3Df>::iterator modulesIt = modules.begin(); modulesIt != modules.end(); modulesIt++)
	{
		m_modules.push_back(std::shared_ptr<Module>(new Module((*modulesIt), totalMass, spaceID, m_bodyID, i++, parameters)));
	}*/

	//for (auto modulesIt = unscaledModules.begin(); modulesIt != unscaledModules.end(); modulesIt++)
	//{
	//	//m_modules.push_back(std::shared_ptr<Module>(new Module(Vector3Df(modulesIt->Getx() * MODULE_SIZE, modulesIt->Gety() * MODULE_SIZE, modulesIt->Getz() * MODULE_SIZE), totalMass, spaceID, m_bodyID, i++, parameters)));

	//	m_moduleMap[*modulesIt] = m_modules.back();
	//}

	//Setup faces
	std::vector<dGeomID> moduleGeoms;

	//Create geom list if required
	if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{
		for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt++)
		{
			moduleGeoms.push_back((*modIt)->GetGeom());
		}
	}

	//TODO: This can be inaccurate and allow the wrong number of faces to be added


	for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
	{
		//For a cubic robot, add faces to modules which lie on AABB

		if (m_parameters->m_robotType == CUBIC)
		{
			if ((*modIt)->GetPosition().Getx() >= MODULE_SIZE * (m_parameters->m_cbRtModules * 0.5 - 1))
			{
				std::shared_ptr<Face> newFace;

				//Create a geom if required
				if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(MODULE_SIZE, 0, 0), Vector3Df(MODULE_SIZE, 0, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), moduleGeoms, true, parameters));

				else
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(MODULE_SIZE, 0, 0), Vector3Df(MODULE_SIZE, 0, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, true, parameters));				

				(*modIt)->AddFace(newFace);

				m_faces.push_back(newFace);

				m_numExternalThrusters ++;
			}

			if ((*modIt)->GetPosition().Getx() <= -MODULE_SIZE * (m_parameters->m_cbRtModules * 0.5 - 1))
			{
				std::shared_ptr<Face> newFace;

				//Create a geom if required
				if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(-MODULE_SIZE, 0, 0), Vector3Df(-MODULE_SIZE, 0, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), moduleGeoms, true, parameters));

				else
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(-MODULE_SIZE, 0, 0), Vector3Df(-MODULE_SIZE, 0, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, true, parameters));				

				(*modIt)->AddFace(newFace);

				m_faces.push_back(newFace);

				m_numExternalThrusters ++;
			}

			if ((*modIt)->GetPosition().Gety() >= MODULE_SIZE * (m_parameters->m_cbRtModules * 0.5 - 1))
			{
				std::shared_ptr<Face> newFace;

				//Create a geom if required
				if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(0, MODULE_SIZE, 0), Vector3Df(0, MODULE_SIZE, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), moduleGeoms, true, parameters));

				else
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(0, MODULE_SIZE, 0), Vector3Df(0, MODULE_SIZE, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, true, parameters));				

				(*modIt)->AddFace(newFace);

				m_faces.push_back(newFace);

				m_numExternalThrusters ++;
			}

			if ((*modIt)->GetPosition().Gety() <= -MODULE_SIZE * (m_parameters->m_cbRtModules * 0.5 - 1))
			{
				std::shared_ptr<Face> newFace;

				//Create a geom if required
				if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM)
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(0, -MODULE_SIZE, 0), Vector3Df(0, -MODULE_SIZE, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), moduleGeoms, true, parameters));

				else
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(0, -MODULE_SIZE, 0), Vector3Df(0, -MODULE_SIZE, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, true, parameters));				

				(*modIt)->AddFace(newFace);

				m_faces.push_back(newFace);

				m_numExternalThrusters ++;
			}

			if ((*modIt)->GetPosition().Getz() >= MODULE_SIZE * (m_parameters->m_cbRtModules * 0.5 - 1))
			{
				std::shared_ptr<Face> newFace;

				//Create a geom if required
				if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM)
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(0, 0, MODULE_SIZE), Vector3Df(0, 0, MODULE_SIZE), totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), moduleGeoms, true, parameters));

				else
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(0, 0, MODULE_SIZE), Vector3Df(0, 0, MODULE_SIZE), totalMass, spaceID, obstacleSpaceID, m_bodyID, true, parameters));				

				(*modIt)->AddFace(newFace);

				m_faces.push_back(newFace);

				m_numExternalThrusters ++;
			}

			if ((*modIt)->GetPosition().Getz() <= -MODULE_SIZE * (m_parameters->m_cbRtModules * 0.5 - 1))
			{
				std::shared_ptr<Face> newFace;

				//Create a geom if required
				if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM)
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(0, 0, -MODULE_SIZE), Vector3Df(0, 0, -MODULE_SIZE), totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), moduleGeoms, true, parameters));

				else
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(0, 0, -MODULE_SIZE), Vector3Df(0, 0, -MODULE_SIZE), totalMass, spaceID, obstacleSpaceID, m_bodyID, true, parameters));				

				(*modIt)->AddFace(newFace);

				m_faces.push_back(newFace);

				m_numExternalThrusters ++;
			}
		}

		else
		{
			//std::cout << "Module position: " << (*modIt)->GetPosition().Getx() << " " << (*modIt)->GetPosition().Gety() << " " << (*modIt)->GetPosition().Getz() << std::endl;

			Vector3Df normals[6] = {Vector3Df(MODULE_SIZE, 0, 0), Vector3Df(-MODULE_SIZE, 0, 0), Vector3Df(0, MODULE_SIZE, 0), Vector3Df(0, -MODULE_SIZE, 0), Vector3Df(0, 0, MODULE_SIZE), Vector3Df(0, 0, -MODULE_SIZE)};

			//Check each of the six cardinal directions around each module to see if there's another module there. If there isn't, add an external face. Else, add an internal face
			for (int i = 0; i != 6; i ++)
			{
				bool external = true;

				Vector3Df neighbourPoint = (*modIt)->GetPosition() + normals[i];

				for (auto modIt2 = m_modules.begin(); modIt2 != m_modules.end(); modIt2 ++)
				{
					//Check if point is within another moudle. If it is - internal face.
					if (neighbourPoint.Getx() < (*modIt2)->GetPosition().Getx() + 0.5 * MODULE_SIZE &&
						neighbourPoint.Getx() > (*modIt2)->GetPosition().Getx() - 0.5 * MODULE_SIZE &&
						neighbourPoint.Gety() < (*modIt2)->GetPosition().Gety() + 0.5 * MODULE_SIZE &&
						neighbourPoint.Gety() > (*modIt2)->GetPosition().Gety() - 0.5 * MODULE_SIZE &&
						neighbourPoint.Getz() < (*modIt2)->GetPosition().Getz() + 0.5 * MODULE_SIZE &&
						neighbourPoint.Getz() > (*modIt2)->GetPosition().Getz() - 0.5 * MODULE_SIZE)
					{
						external = false;
						break;
					}

				}



				if (external)
				{
					//std::cout << m_numExternalThrusters << std::endl;

					//std::cout << (*modIt)->GetPosition().Getx() + 0.5 * normals[i].Getx() << " " << (*modIt)->GetPosition().Gety() + 0.5 * normals[i].Gety() << " " << (*modIt)->GetPosition().Getz() + 0.5 * normals[i].Getz() << std::endl;

					std::shared_ptr<Face> newFace;

					//Create a geom if required
					if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
					{
						newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * normals[i], normals[i], totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), moduleGeoms, true, parameters));
					}

					else
					{
						newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * normals[i], normals[i], totalMass, spaceID, obstacleSpaceID, m_bodyID, true, parameters));
					}

				

					(*modIt)->AddFace(newFace);

					m_faces.push_back(newFace);

					m_numExternalThrusters ++;
				}

				/*else
				{
					std::shared_ptr<Face> newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * normals[i], normals[i], m_targetPosition, totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), false, parameters));

					(*modIt)->AddFace(newFace);

					m_faces.push_back(newFace);
				}*/
			}
		}
	}

	//while(true);

	std::cout << "EXT: " << m_numExternalThrusters << std::endl;

	//dMassTranslate (&totalMass, -totalMass.c[0], -totalMass.c[1], -totalMass.c[2]);

	dBodySetMass (m_bodyID, &totalMass);

	dBodySetPosition(m_bodyID, startingPosition.Getx(), startingPosition.Gety(), startingPosition.Getz());


	if (m_parameters->m_controller == CENTRALIZED)
		CalculatePseudoInverse();


	//Set body rotation
	dMatrix3 dRotation =
	{
		startingOrientation.GetElement(0, 0), startingOrientation.GetElement(0, 1), startingOrientation.GetElement(0, 2), 0,
		startingOrientation.GetElement(1, 0), startingOrientation.GetElement(1, 1), startingOrientation.GetElement(1, 2), 0,
		startingOrientation.GetElement(2, 0), startingOrientation.GetElement(2, 1), startingOrientation.GetElement(2, 2), 0
	};

	dBodySetRotation(m_bodyID, dRotation);

	//Set AABB of robot

	m_AABB.push_back(m_modules.back()->GetPosition().Getx());
	m_AABB.push_back(m_modules.back()->GetPosition().Getx());
	m_AABB.push_back(m_modules.back()->GetPosition().Gety());
	m_AABB.push_back(m_modules.back()->GetPosition().Gety());
	m_AABB.push_back(m_modules.back()->GetPosition().Getz());
	m_AABB.push_back(m_modules.back()->GetPosition().Getz());

	for (auto mod = m_modules.begin(); mod != m_modules.end(); mod++)
	{
		if ((*mod)->GetPosition().Getx() < m_AABB[0])
			m_AABB[0] = (*mod)->GetPosition().Getx();

		if ((*mod)->GetPosition().Getx() > m_AABB[1])
			m_AABB[1] = (*mod)->GetPosition().Getx();

		if ((*mod)->GetPosition().Gety() < m_AABB[2])
			m_AABB[2] = (*mod)->GetPosition().Gety();

		if ((*mod)->GetPosition().Gety() > m_AABB[3])
			m_AABB[3] = (*mod)->GetPosition().Gety();

		if ((*mod)->GetPosition().Getz() < m_AABB[4])
			m_AABB[4] = (*mod)->GetPosition().Getz();

		if ((*mod)->GetPosition().Getz() > m_AABB[5])
			m_AABB[5] = (*mod)->GetPosition().Getz();
	}

	m_AABB[0] -= MODULE_SIZE / 2;
	m_AABB[1] += MODULE_SIZE / 2;
	m_AABB[2] -= MODULE_SIZE / 2;
	m_AABB[3] += MODULE_SIZE / 2;
	m_AABB[4] -= MODULE_SIZE / 2;
	m_AABB[5] += MODULE_SIZE / 2;

	SetLinkedFaces();
}

//Deconstructor (must remove all ODE bodies etc.)
Robot::~Robot()
{
	dBodyDestroy(m_bodyID);
}

void Robot::SetStartingOrientation()
{
	if(m_parameters->m_randomOrientation)
	{
		bool finished = false;

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


		//Set body rotation
		const dMatrix3 dRotation =
		{
			rot.GetElement(0, 0), rot.GetElement(0, 1), rot.GetElement(0, 2), 0,
			rot.GetElement(1, 0), rot.GetElement(1, 1), rot.GetElement(1, 2), 0,
			rot.GetElement(2, 0), rot.GetElement(2, 1), rot.GetElement(2, 2), 0
		};

		dBodySetRotation(m_bodyID, dRotation);
	}

	else
	{
		//const dMatrix3 dRotation =
		//{
		//	1, 0, 0, 0,
		//	0, 1, 0, 0,
		//	0, 0, 1, 0
		//};

		//Vertex on

		float theta = PI / float(4);
		float psi = atan(float(1) / float(sqrt(float(2))));
		float phi = 0;

		const dMatrix3 dRotation =
		{
			cos(theta) * cos(psi), -sin(psi), sin(theta) * cos(psi), 0,
			cos(theta) * sin(psi), cos(psi), sin(theta) * sin(psi), 0,
			-sin(theta), 0, cos(theta), 0
		};

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

		dBodySetRotation(m_bodyID, dRotation);
	}
}

void Robot::ControlStep(Vector3Df targetPosition)
{
	//Set thrusters to fire or not

	//Run centralized controller or occlusion controller
	if (m_parameters->m_controller == CENTRALIZED)
	{
		//TRANSLATION

		//Get target position in body frame
		dVector3 target;

		dBodyGetPosRelPoint(m_bodyID, targetPosition[0], targetPosition[1], targetPosition[2], target);

		//Get translation error vector
		MatrixNMf eTrans(3, 1);
		
		for (int i = 0; i != 3; i ++)
			eTrans.SetElement(i, 0, target[i]);

		//Calculate thruster error vector
		MatrixNMf thrusterErrors = m_transPseudoInverse.MatrixMultiply(eTrans);

		//Now use PID translational controller for each thruster

		int i = 0;

		for (auto moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt++)
		{
			std::vector<std::shared_ptr<Face>> faces = (*moduleIt)->GetFaces();

			for (std::vector<std::shared_ptr<Face>>::iterator faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
			{
				(*faceIt)->RunTransPID(thrusterErrors.GetElement(i, 0));

				//(*faceIt)->CollatePIDs();

				i ++;
			}
		}

		////ROTATION
		////Get target vector in robot coordinates
		//dVector3 globalTarget = {targetPosition[0] - GetPosition()[0], targetPosition[1] - GetPosition()[1], targetPosition[2] - GetPosition()[2]};
		//dVector3 localTarget;
		//dBodyVectorFromWorld(m_bodyID, globalTarget[0], globalTarget[1], globalTarget[2], localTarget);

		//Vector3Df normalTarget(localTarget);
		//normalTarget.Normalize();

		//MatrixNMf eRot(3, 1);

		////If the target vector and the (1, 0, 0) vector are facing in the opposite direction you can't take the cross product and need to choose an axis to rotate around manually

		//if (normalTarget.Getx() == 1)
		//{
		//	eRot.SetElement(0, 0, 0);
		//	eRot.SetElement(1, 0, 0);
		//	eRot.SetElement(2, 0, PI);
		//}

		//else
		//{
		//	float angle = acos(normalTarget.DotWith(Vector3Df(-1, 0, 0)));

		//	Vector3Df r(0, normalTarget.Getz(),-normalTarget.Gety());

		//	r.Normalize();

		//	eRot.SetElement(0, 0, 0);
		//	eRot.SetElement(1, 0, r.Gety() * angle);
		//	eRot.SetElement(2, 0, r.Getz() * angle);
		//}

		////Calculate thruster error vector
		//MatrixNMf thrusterRotErrors = m_rotPseudoInverse.MatrixMultiply(eRot);

		////Now use PID controllers
		//i = 0;
		//for (auto moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt++)
		//{
		//	std::vector<std::shared_ptr<Face>> faces = (*moduleIt)->GetFaces();
		//	for (std::vector<std::shared_ptr<Face>>::iterator faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
		//	{
		//		(*faceIt)->RunRotPID(thrusterRotErrors.GetElement(i, 0));
		//		(*faceIt)->CollatePIDs();
		//		i ++;
		//	}
		//}



		//ROTATION

		//Get target vector in robot coordinates
		dVector3 globalTarget = {targetPosition[0] - GetPosition()[0], targetPosition[1] - GetPosition()[1], targetPosition[2] - GetPosition()[2]};

		dVector3 localTarget;

		dBodyVectorFromWorld(m_bodyID, globalTarget[0], globalTarget[1], globalTarget[2], localTarget);

		Vector3Df normalTarget(localTarget);

		normalTarget.Normalize();

		float num = float(1)/sqrt(float(3));

		Vector3Df vertexDirections[8] = {Vector3Df(num, num, num), Vector3Df(num, num, -num), Vector3Df(num, -num, num), Vector3Df(num, -num, -num), Vector3Df(-num, num, num), Vector3Df(-num, num, -num), Vector3Df(-num, -num, num), Vector3Df(-num, -num, -num)};

		Vector3Df direction(0, 0, 0);
		float smallestAngle = PI / float(2);

		//Go through each of 6 directions, find which one make the smallest angle with the target vector
		for (int i = 0; i != 6; i ++)
		{
			//float angle = acos(normalTarget.DotWith(directions[i]));
			float angle = acos(normalTarget.DotWith(vertexDirections[i]));

			if (angle < smallestAngle)
			{
				smallestAngle = angle;
				//direction = directions[i];
				direction = vertexDirections[i];
			}
		}

		//Now get rotation axis
		//Vector3Df rotationAxis = normalTarget.CrossWith(direction);
		Vector3Df rotationAxis = direction.CrossWith(normalTarget);

		//Calculate torque error vector
		MatrixNMf eRot(3, 1);

	
		Vector3Df r(rotationAxis[0], rotationAxis[1], rotationAxis[2]);

		r.Normalize();

		eRot.SetElement(0, 0, r.Getx() * smallestAngle);
		eRot.SetElement(1, 0, r.Gety() * smallestAngle);
		eRot.SetElement(2, 0, r.Getz() * smallestAngle);
		
		/*for (int i = 0; i != 3; i ++)
			eRot.SetElement(i, 0, smallestAngle * rotationAxis[i]);*/

		//Calculate thruster error vector
		MatrixNMf thrusterRotErrors = m_rotPseudoInverse.MatrixMultiply(eRot);

		//Now use PID controllers

		i = 0;

		for (auto moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt++)
		{
			std::vector<std::shared_ptr<Face>> faces = (*moduleIt)->GetFaces();

			for (std::vector<std::shared_ptr<Face>>::iterator faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
			{
				(*faceIt)->RunRotPID(thrusterRotErrors.GetElement(i, 0));

				(*faceIt)->CollatePIDs();

				i ++;
			}
		}
	}

	else
	{
		for (auto moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt ++)
			(*moduleIt)->Step(targetPosition);
	}

	//Turn off redundant thrusters if communication is enabled

	if (m_parameters->m_controller == DECENTRALIZED_COMM)
	{
		for (auto moduleIt = m_moduleMap.begin(); moduleIt != m_moduleMap.end(); moduleIt ++)
		{
			std::vector<std::shared_ptr<Face>> faces = moduleIt->second->GetFaces();

			for (auto faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
			{
				if ((*faceIt)->GetThrusterFiring() && (*faceIt)->GetLinkedFace()->GetThrusterFiring())
				{
					(*faceIt)->SetThrusterFiring(false);
					(*faceIt)->GetLinkedFace()->SetThrusterFiring(false);
				}
			}
		}
	}


		////If using communication, check if opposing thrusters are firing and turn them off if they are
		//for (auto moduleIt = m_moduleMap.begin(); moduleIt != m_moduleMap.end(); moduleIt ++)
		//{
		//	std::vector<std::shared_ptr<Face>> faces = moduleIt->second->GetFaces();

		//	for (auto faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
		//	{
		//		//Check if thruster is firing
		//		if ((*faceIt)->GetThrusterFiring())
		//		{
		//			bool faceFound = false;

		//			int i = 0;
		//			//if it is, find the opposing thruster
		//			while (!faceFound)
		//			{
		//				Vector3Di antiNormal = Vector3Di((*faceIt)->GetNormal().Getx(), (*faceIt)->GetNormal().Gety(), (*faceIt)->GetNormal().Getz());

		//				//Iterate through line of modules away from face until you find one that has the opposing face

		//				//std::vector<std::shared_ptr<Face>> nextFaces = m_moduleMap[moduleIt->first + (i * antiNormal)]->GetFaces();

		//				Vector3Di index = Vector3Di(moduleIt->first) + i * antiNormal;

		//				std::vector<std::shared_ptr<Face>> nextFaces = m_moduleMap[index]->GetFaces();

		//				for (auto faceIt2 = nextFaces.begin(); faceIt2 != nextFaces.end(); faceIt2 ++)
		//				{
		//					if (Vector3Di((*faceIt2)->GetNormal().Getx(), (*faceIt2)->GetNormal().Gety(), (*faceIt2)->GetNormal().Getz()) == antiNormal)	
		//					{
		//						faceFound = true;

		//						//If both thrusters are firing, turn them off
		//						if ((*faceIt2)->GetThrusterFiring())
		//						{
		//							(*faceIt)->SetThrusterFiring(false);
		//							(*faceIt2)->SetThrusterFiring(false);
		//						}
		//					}
		//				}

		//				i++;
		//			}
		//		}
		//	}
		//}
	//}

	//Count number of firing thrusters
	for (auto module = m_modules.begin(); module != m_modules.end(); module ++)
		m_thrusterFiringCounter += (*module)->GetNumFiringThrusters();
}

void Robot::Draw()
{
	for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
		(*modIt)->Draw();
}

void Robot::ApplyThrusterForces()
{
	for (auto moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt ++)
	{
		std::vector<std::shared_ptr<Face>> faces = (*moduleIt)->GetFaces();

		for (auto faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
		{
			(*faceIt)->FireThruster();
		}
	}
}

void Robot::ApplyDrag()
{
	//Iterate over every external face in the robot
	//Get the velocity of the face
	//Check that the velocity is pointing 'outwards'
	//Get the perpendicular component of this and use it to add a force in the opposite direction
	//Fperp = const . Vperp . Vperp

	//return;

	//QUADRATIC DRAG
	if (m_parameters->m_dragType == QUADRATIC)
	{
		for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
		{
			dVector3 globalVelocity;

			//This gets a GLOBAL velocity vector
			dBodyGetRelPointVel(m_bodyID, faceIt->lock()->GetPosition()[0], faceIt->lock()->GetPosition()[1], faceIt->lock()->GetPosition()[2], globalVelocity);

			dVector3 localVelocity;

			dBodyVectorFromWorld(m_bodyID, globalVelocity[0], globalVelocity[1], globalVelocity[2], localVelocity);

			Vector3Df velocity(localVelocity[0], localVelocity[1], localVelocity[2]);

			Vector3Df normalVelocity = velocity;

			normalVelocity.Normalize();

			//Get angle between face normal and velocity
			float cosTheta = velocity.DotWith(faceIt->lock()->GetNormal()) / (sqrt(velocity.ModulusSquared() * faceIt->lock()->GetNormal().ModulusSquared()));

			//Get effective area of module
			float effectiveArea = MODULE_SIZE * MODULE_SIZE * cosTheta;

			//Check if velocity is pointing outwards
			if (acos(cosTheta) < PI / 2)
			{
				Vector3Df force = -1 * FRICTION_CONSTANT * FLUID_DENSITY * effectiveArea * velocity.ModulusSquared() * normalVelocity;

				//If it is, apply friction
				dBodyAddRelForceAtRelPos(m_bodyID, force[0], force[1], force[2], faceIt->lock()->GetPosition()[0], faceIt->lock()->GetPosition()[1], faceIt->lock()->GetPosition()[2]);

			}
		}	
	}

	//LINEAR DRAG
	else if (m_parameters->m_dragType == LINEAR)
	{
		//Stoke's law approximation
		// F = -6 * pi * visc. * r * v

		//Radius, from V = 4/3 * pi * r^3
		float radius = 0.62 * m_parameters->m_cbRtModules * m_parameters->m_moduleSize;

		float constant = radius * 6 * PI * m_parameters->m_fluidViscosity;

		dVector3 globalVelocity;

		dBodyGetRelPointVel(m_bodyID, 0, 0, 0, globalVelocity);

		dVector3 globalForce;

		globalForce[0] = -constant * globalVelocity[0];
		globalForce[1] = -constant * globalVelocity[1];
		globalForce[2] = -constant * globalVelocity[2];


		dBodyAddForceAtRelPos(m_bodyID, globalForce[0], globalForce[1], globalForce[2], 0, 0, 0);
	}
}

Vector3Df Robot::GetPosition()
{
	const float* pos = dBodyGetPosition(m_bodyID);

	return Vector3Df(pos[0], pos[1], pos[2]);
}


void Robot::CalculatePseudoInverse()
{
	//Calculate Moore-Penrose pseudo-inverse
	MatrixNMf thrusterOrientations(3, m_numExternalThrusters);

	MatrixNMf thrusterTorques(3, m_numExternalThrusters);

	int k = 0;

	//Go through each thruster, add the orientation to Arot
	for (auto moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt++)
	{
		std::vector<std::shared_ptr<Face>> faces = (*moduleIt)->GetFaces();

		for (std::vector<std::shared_ptr<Face>>::iterator faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
		{
			Vector3Df orientationVector = -1 * (*faceIt)->GetNormal();

			Vector3Df positionVector = (*faceIt)->GetPosition();

			thrusterOrientations.Set3Column(k, orientationVector);

			thrusterTorques.Set3Column(k, positionVector.CrossWith(orientationVector));

			k++;
		}
	}

	//Now perform the matrix multiplication
	m_transPseudoInverse = thrusterOrientations.GetTranspose().MatrixMultiply(thrusterOrientations.MatrixMultiply(thrusterOrientations.GetTranspose()).GetInverse());

	m_rotPseudoInverse = thrusterTorques.GetTranspose().MatrixMultiply(thrusterTorques.MatrixMultiply(thrusterTorques.GetTranspose()).GetInverse());
}

void Robot::SetLinkedFaces()
{
	////If using communication, check if opposing thrusters are firing and turn them off if they are
	//for (auto moduleIt = m_moduleMap.begin(); moduleIt != m_moduleMap.end(); moduleIt ++)
	//{
	//	std::vector<std::shared_ptr<Face>> faces = moduleIt->second->GetFaces();

	//	std::cout << std::endl;

	//	std::cout << "MODULE: " << moduleIt->first.Getx() << " " << moduleIt->first.Gety() << " " << moduleIt->first.Getz() << std::endl;

	//	for (auto faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
	//	{
	//		Vector3Di normal = Vector3Di(-(*faceIt)->GetNormal().Getx(), -(*faceIt)->GetNormal().Gety(), -(*faceIt)->GetNormal().Getz());

	//		//std::cout << "FACE POS: " << (*faceIt)->GetPosition().Getx() << " " << (*faceIt)->GetPosition().Gety() << " " << (*faceIt)->GetPosition().Getz() << std::endl;

	//		std::cout << "NORMAL: " << (*faceIt)->GetNormal().Getx() << " " << (*faceIt)->GetNormal().Gety() << " " << (*faceIt)->GetNormal().Getz() << std::endl;

	//	}
	//}

	//If using communication, check if opposing thrusters are firing and turn them off if they are
	for (auto moduleIt = m_moduleMap.begin(); moduleIt != m_moduleMap.end(); moduleIt ++)
	{
		std::vector<std::shared_ptr<Face>> faces = moduleIt->second->GetFaces();

		//std::cout << std::endl;

		for (auto faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
		{
			Vector3Di normal = Vector3Di(-(*faceIt)->GetNormal().Getx(), -(*faceIt)->GetNormal().Gety(), -(*faceIt)->GetNormal().Getz());

			//std::cout << "MODULE: " << moduleIt->first.Getx() << " " << moduleIt->first.Gety() << " " << moduleIt->first.Getz() << std::endl;

			//std::cout << "FACE POS: " << (*faceIt)->GetPosition().Getx() << " " << (*faceIt)->GetPosition().Gety() << " " << (*faceIt)->GetPosition().Getz() << std::endl;

			//std::cout << "NORMAL: " << (*faceIt)->GetNormal().Getx() << " " << (*faceIt)->GetNormal().Gety() << " " << (*faceIt)->GetNormal().Getz() << std::endl;

			bool linkFound = false;

			int i = 0;

			while (!linkFound)
			{
				//std::cout << i << std::endl;

				//std::cout << "test: " << Vector3Di(moduleIt->first).Getx() + i * normal.Getx() << " " << Vector3Di(moduleIt->first).Gety() + i * normal.Gety() << " " << Vector3Di(moduleIt->first).Getz() + i * normal.Getz() << std::endl;

				if (!m_moduleMap.count(Vector3Di(moduleIt->first) + i * normal))
					linkFound = true;

				i ++;

				//std::cout << i << std::endl;
			}

			//std::cout << "i: " << i << std::endl;

			//std::cout << "2nd MODULE: " << Vector3Di(moduleIt->first).Getx() + (i - 2) * normal.Getx() << " " << Vector3Di(moduleIt->first).Gety() + (i - 2) * normal.Gety() << " " << Vector3Di(moduleIt->first).Getz() + (i - 2) * normal.Getz() << std::endl;

			/*if (m_moduleMap.count(Vector3Di(moduleIt->first) + (i - 2) * normal))
			{*/
				std::vector<std::shared_ptr<Face>> faces2 = m_moduleMap[Vector3Di(moduleIt->first) + (i - 2) * normal]->GetFaces();

				for (auto face2It = faces2.begin(); face2It != faces2.end(); face2It ++)
				{
					//std::cout << "2nd NORMAL " << (*face2It)->GetNormal().Getx() << " " <<(*face2It)->GetNormal().Gety() << " " << (*face2It)->GetNormal().Getz() << std::endl;

					if ((*face2It)->GetNormal() == -1 * (*faceIt)->GetNormal())
					{
						(*faceIt)->SetLinkedface(*face2It);

						//std::cout << "LINK: " << (*faceIt)->GetPosition().Getx() << " " << (*faceIt)->GetPosition().Gety() << " " << (*faceIt)->GetPosition().Getz() << "    " << (*face2It)->GetPosition().Getx() << " " << (*face2It)->GetPosition().Gety() << " " << (*face2It)->GetPosition().Getz() << std::endl;
					}
				}
			//}
		}
	}
}