#include "Robot.hpp"


//Constructor
Robot::Robot(dWorldID& worldID, dSpaceID& spaceID, dSpaceID& obstacleSpaceID, std::vector<Vector3Df> modules, std::vector<Vector3Di> unscaledModules, Vector3Df startingPosition, MatrixNMf startingOrientation, std::shared_ptr<Parameters> parameters) : m_thrusterFiringCounter(0)
{
	//Parameters
	m_parameters = parameters;

	//Set up ODE body
	m_bodyID = dBodyCreate(worldID);

	//Set ODE space ID
	m_robotSpaceID = spaceID;

	m_numExternalThrusters = 0;

	m_collisionCounter = 0;

	//Set up mass distribution
	dMass totalMass;

	//For the robots of random and convex topology, create a mass box for each of the modules
	if (m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{
		dMass geomMass;

		//Go through modules and properly add each ones mass to the total mass
		for (std::vector<Vector3Df>::iterator modulesIt = modules.begin(); modulesIt != modules.end(); modulesIt++)
		{
			dMassSetZero (&geomMass);

			dMassSetBox(&geomMass, ROBOT_DENSITY, MODULE_SIZE, MODULE_SIZE, MODULE_SIZE);

			//Move the mass into the correct position
			dMassTranslate(&geomMass, modulesIt->Getx(), modulesIt->Gety(), modulesIt->Getz());

			dMassAdd(&totalMass, &geomMass);
		}
	}

	//For a cubic robot, a single mass box the size of the robot can be used
	else
	{
		dMassSetBox(&totalMass, ROBOT_DENSITY, MODULE_SIZE * m_parameters->m_cbRtModules, MODULE_SIZE * m_parameters->m_cbRtModules, MODULE_SIZE * m_parameters->m_cbRtModules);
	}

	//Add the module objects to the robot
	for (int i = 0; i != modules.size(); i ++)
	{
		m_modules.push_back(std::shared_ptr<Module>(new Module(modules[i], totalMass, spaceID, m_bodyID, i, parameters)));

		m_moduleMap[unscaledModules[i]] = m_modules.back();
	}

	
	//Get created module geoms from the modules if required by the scenario (allows collisions)
	std::vector<dGeomID> moduleGeoms;

	if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{
		for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt++)
		{
			moduleGeoms.push_back((*modIt)->GetGeom());
		}
	}

	//Add external module faces

	for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
	{
		//For a cubic robot, add faces to modules which lie on the AABB
		if (m_parameters->m_robotType == CUBIC)
		{
			//Positive x boundary
			if ((*modIt)->GetPosition().Getx() >= MODULE_SIZE * (m_parameters->m_cbRtModules * 0.5 - 1))
			{
				std::shared_ptr<Face> newFace;

				//Create a face geom if required
				if (m_parameters->m_scenario == OBSTACLES || m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(MODULE_SIZE, 0, 0), Vector3Df(MODULE_SIZE, 0, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, (*modIt)->GetGeom(), moduleGeoms, true, parameters));

				else
					newFace = std::shared_ptr<Face>(new Face((*modIt)->GetPosition() + 0.5 * Vector3Df(MODULE_SIZE, 0, 0), Vector3Df(MODULE_SIZE, 0, 0), totalMass, spaceID, obstacleSpaceID, m_bodyID, true, parameters));				

				(*modIt)->AddFace(newFace);

				m_faces.push_back(newFace);

				//Increment the number of thrusters (one per external face)
				m_numExternalThrusters ++;
			}

			//Negative x boundary
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

			//Positive y boundary
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

			//Negative y boundary
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

			//Positive z boundary
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

			//Negative z boundary
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

		//For non-cubic robots, iterate through each module and add external faces
		else
		{
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

				//If the face is external, create it, including geom if required
				if (external)
				{

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
			}
		}
	}

	//Set the created mass as the robots mass
	dBodySetMass (m_bodyID, &totalMass);

	//Set the robot at the starting position
	dBodySetPosition(m_bodyID, startingPosition.Getx(), startingPosition.Gety(), startingPosition.Getz());

	//Set the starting rotation of the robot
	dMatrix3 dRotation =
	{
		startingOrientation.GetElement(0, 0), startingOrientation.GetElement(0, 1), startingOrientation.GetElement(0, 2), 0,
		startingOrientation.GetElement(1, 0), startingOrientation.GetElement(1, 1), startingOrientation.GetElement(1, 2), 0,
		startingOrientation.GetElement(2, 0), startingOrientation.GetElement(2, 1), startingOrientation.GetElement(2, 2), 0
	};

	dBodySetRotation(m_bodyID, dRotation);

	//Calculate the pseudoinverse matrices if using the centralized controller
	if (m_parameters->m_controller == CENTRALIZED)
		CalculatePseudoInverse();

	//Set the AABB of robot
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

	//Creates link between faces on either end of an unbroken line of modules, if communication is being used
	SetLinkedFaces();
}

//Deconstructor (must remove all ODE bodies)
Robot::~Robot()
{
	dBodyDestroy(m_bodyID);
}

//Update the robot controller
void Robot::ControlStep(Vector3Df targetPosition)
{
	//Run centralized controller if required
	//https://dspace.mit.edu/bitstream/handle/1721.1/67312/Rus_Complete%20SE.pdf?sequence=1&isAllowed=y
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

				i ++;
			}
		}

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
			float angle = acos(normalTarget.DotWith(vertexDirections[i]));

			if (angle < smallestAngle)
			{
				smallestAngle = angle;
				direction = vertexDirections[i];
			}
		}

		//Now get rotation axis
		Vector3Df rotationAxis = direction.CrossWith(normalTarget);

		//Calculate torque error vector
		MatrixNMf eRot(3, 1);
	
		Vector3Df r(rotationAxis[0], rotationAxis[1], rotationAxis[2]);

		r.Normalize();

		eRot.SetElement(0, 0, r.Getx() * smallestAngle);
		eRot.SetElement(1, 0, r.Gety() * smallestAngle);
		eRot.SetElement(2, 0, r.Getz() * smallestAngle);
		
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

	//If not using centralized controller, control is hanlded in a distributed manner
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

	//Count number of firing thrusters
	for (auto module = m_modules.begin(); module != m_modules.end(); module ++)
		m_thrusterFiringCounter += (*module)->GetNumFiringThrusters();
}

//Draw the robot
void Robot::Draw()
{
	for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
		(*modIt)->Draw();
}

//Fire those robot thrusters that the controller has determiend should be active this step
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

//Apply fluid drag to the robot. Both linear and quadratic options
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

	//LINEAR DRAG - treat the robot as a sphere and add the drag force at its center
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

//Return the position of the robot as a Vector3Df
Vector3Df Robot::GetPosition()
{
	const float* pos = dBodyGetPosition(m_bodyID);

	return Vector3Df(pos[0], pos[1], pos[2]);
}

//Calculate the translational and rotational pseudoinverses for the centralized controller
//https://dspace.mit.edu/bitstream/handle/1721.1/67312/Rus_Complete%20SE.pdf?sequence=1&isAllowed=y
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

//Creates link between faces on either end of an unbroken line of modules, if communication is being used
void Robot::SetLinkedFaces()
{
	//If using communication, check if opposing thrusters are firing and turn them off if they are
	for (auto moduleIt = m_moduleMap.begin(); moduleIt != m_moduleMap.end(); moduleIt ++)
	{
		std::vector<std::shared_ptr<Face>> faces = moduleIt->second->GetFaces();

		for (auto faceIt = faces.begin(); faceIt != faces.end(); faceIt ++)
		{
			//Get the face anti-normal
			Vector3Di normal = Vector3Di(-(*faceIt)->GetNormal().Getx(), -(*faceIt)->GetNormal().Gety(), -(*faceIt)->GetNormal().Getz());

			bool linkFound = false;

			int i = 0;

			//Iterate along the anti normal until a gap is found, and remember the value of the iterator i
			while (!linkFound)
			{
				if (!m_moduleMap.count(Vector3Di(moduleIt->first) + i * normal))
					linkFound = true;

				i ++;
			}

			//Get the faces of the last module before the gap
			std::vector<std::shared_ptr<Face>> faces2 = m_moduleMap[Vector3Di(moduleIt->first) + (i - 2) * normal]->GetFaces();

			for (auto face2It = faces2.begin(); face2It != faces2.end(); face2It ++)
			{
				//Find the face on the discovered module that has an opposite normal to the face on the starting module
				if ((*face2It)->GetNormal() == -1 * (*faceIt)->GetNormal())
				{
					//Link the faces
					(*faceIt)->SetLinkedface(*face2It);		
				}
			}
		}
	}
}