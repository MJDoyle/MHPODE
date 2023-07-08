#include "Face.hpp"

Face::Face(Vector3Df position, Vector3Df normal, dMass mass, dSpaceID& robotSpace, dSpaceID& obstacleSpace, dBodyID& body, dGeomID& geom, std::vector<dGeomID> moduleGeoms, bool external, std::shared_ptr<Parameters> parameters) 
{
	m_parameters = parameters;
	m_position = position; 
	m_unNormal = normal;
	m_normal = 1 / MODULE_SIZE * normal; 


	//std::cout << "FACE NORMAL: " << m_normal.Getx() << " " << m_normal.Gety() << " " << m_normal.Getz() << std::endl;

	m_robotSpaceID = robotSpace;
	m_obstacleSpaceID = obstacleSpace;
	m_joint = false;
	m_PIDpreviousErrorTrans = 0;
	m_PIDoutputTrans = 0;
	m_PIDintegralTrans = 0;
	m_PIDpreviousErrorRot = 0;
	m_PIDoutputRot = 0;
	m_PIDintegralRot = 0;

	m_workingThruster = true;

	if (parameters->m_thrusterFailureRate > 0)
	{
		if (RandFloat() < parameters->m_thrusterFailureRate)
			m_workingThruster = false;
	}

	m_thrusterStateChanged = false;

	m_moduleGeoms = moduleGeoms;

	//std::cout << "Face position: " << m_position[0] << " " << m_position[1] << " " << m_position[2] << std::endl;

	//std::cout << "Face normal: " << m_normal[0] << " " << m_normal[1] << " " << m_normal[2] << std::endl;

	//m_CoMrelativePosition = m_position;

	//m_CoMrelativePosition = Vector3Df(m_position.Getx() - mass.c[0], m_position.Gety() - mass.c[1], m_position.Getz() - mass.c[2]);

	//m_CoMrelativePosition = m_position;

	m_robotBodyID = body;

	m_moduleGeomID = geom;

	m_targetSensorRayGeom = dCreateRay(0, 1000);

	m_obstacleSensorRayGeom = dCreateRay(0, OBSTACLE_SENSOR_RANGE);

	m_thrusterFiring = false;

	m_external = external;

	m_flowID = -1;

	m_flowChecked = false;

	//Sensors not detecteding up to 90 degress
	m_targetDetectionThreshold = float(90 - m_parameters->m_sensorNoise) * float(PI) / float(180);

	std::cout << m_targetDetectionThreshold;
}

Face::Face(Vector3Df position, Vector3Df normal, dMass mass, dSpaceID& robotSpace, dSpaceID& obstacleSpace, dBodyID& body, bool external, std::shared_ptr<Parameters> parameters)
{
	m_parameters = parameters;
	m_position = position; 
	m_unNormal = normal;
	m_normal = 1 / MODULE_SIZE * normal; 


	//std::cout << "FACE NORMAL: " << m_normal.Getx() << " " << m_normal.Gety() << " " << m_normal.Getz() << std::endl;

	m_robotSpaceID = robotSpace;
	m_obstacleSpaceID = obstacleSpace;
	m_joint = false;
	m_PIDpreviousErrorTrans = 0;
	m_PIDoutputTrans = 0;
	m_PIDintegralTrans = 0;
	m_PIDpreviousErrorRot = 0;
	m_PIDoutputRot = 0;
	m_PIDintegralRot = 0;

	m_workingThruster = true;

	if (parameters->m_thrusterFailureRate > 0)
	{
		if (RandFloat() < parameters->m_thrusterFailureRate)
			m_workingThruster = false;
	}

	m_thrusterStateChanged = false;

	//std::cout << "Face position: " << m_position[0] << " " << m_position[1] << " " << m_position[2] << std::endl;

	//std::cout << "Face normal: " << m_normal[0] << " " << m_normal[1] << " " << m_normal[2] << std::endl;

	//m_CoMrelativePosition = m_position;

	//m_CoMrelativePosition = Vector3Df(m_position.Getx() - mass.c[0], m_position.Gety() - mass.c[1], m_position.Getz() - mass.c[2]);

	//m_CoMrelativePosition = m_position;

	m_robotBodyID = body;

	m_targetSensorRayGeom = dCreateRay(0, 1000);

	m_obstacleSensorRayGeom = dCreateRay(0, OBSTACLE_SENSOR_RANGE);

	m_thrusterFiring = false;

	m_external = external;

	m_flowID = -1;

	m_flowChecked = false;

	//Sensors not detecteding up to 90 degress
	m_targetDetectionThreshold = float(90 - m_parameters->m_sensorNoise) * float(PI) / float(180);
}

void Face::Draw()
{
	/*if (m_normal.Getx() < 0)
		dsSetColor(0, 0, 1);*/


	if (m_thrusterFiring && m_workingThruster)
		//dsSetColorAlpha(0, 0.3, 0, 0.5);
		dsSetColor (0, 0.3, 0);

	else
		//dsSetColorAlpha(1, 1, 0.94, 0.5);
		dsSetColor(1, 1, 0.94);

	float delta = MODULE_SIZE / float(40);

	dVector3 S = {MODULE_SIZE - delta, MODULE_SIZE - delta, MODULE_SIZE - delta};
	
	if (m_normal.Getz() != 0)
		S[2] = delta;

	else if (m_normal.Gety() != 0)
		S[1] = delta;

	else if (m_normal.Getx() != 0)
		S[0] = delta;


	dVector3 position;

	//dBodyGetRelPointPos(m_robotBodyID, m_position.Getx(), m_position.Gety(), m_position.Getz(), position);

	dBodyGetRelPointPos(m_robotBodyID, m_position.Getx(), m_position.Gety(), m_position.Getz(), position);

	dsDrawBox (position, dBodyGetRotation(m_robotBodyID), S);
}

bool Face::GetTargetSensorInput(Vector3Df targetPosition)
{
	//Two different methods depending on robot. For a cubic (convex) robot just check angle between face normal and target vector from face center. For arbitrary (concave) robot use the ODE ray casting functinality

	if (m_parameters->m_robotType == RANDOM || m_parameters->m_robotType == CONVEX)
	{

		//Set the ray to the correct position, orientation and length. The position is the center of this face. The orientation is that such that it points at the target position, the length is that such it reaches the target position.

		dVector3 targetInBodyFrame;

		dBodyGetPosRelPoint(m_robotBodyID, targetPosition.Getx(), targetPosition.Gety(), targetPosition.Getz(), targetInBodyFrame);

		Vector3Df targetDelta = Vector3Df(targetInBodyFrame[0], targetInBodyFrame[1], targetInBodyFrame[2]) - m_position;

		targetDelta.Normalize();


		float angle = acos(targetDelta.DotWith(m_normal));

		if (angle > m_targetDetectionThreshold)
		{
			return true;
		}

		//Position
		dVector3 rayPositionInWorldSpace;
	
		dBodyGetRelPointPos(m_robotBodyID, m_position.Getx() + m_normal.Getx() /100000, m_position.Gety() + m_normal.Gety() /100000, m_position.Getz() + m_normal.Getz() /100000, rayPositionInWorldSpace);

		//dBodyGetRelPointPos(m_robotBodyID, m_CoMrelativePosition.Getx(), m_CoMrelativePosition.Gety(), m_CoMrelativePosition.Getz(), rayPositionInWorldSpace);


		//Orientation
		Vector3Df rayOrientation = targetPosition - Vector3Df(rayPositionInWorldSpace);


		//float pos1[3];
		//float pos2[3];

		//pos1[0] = m_targetPosition.Getx();
		//pos1[1] = m_targetPosition.Gety();
		//pos1[2] = m_targetPosition.Getz();

	

		//dsDrawLine(pos1, rayPositionInWorldSpace);

		//Length
		float length = rayOrientation.Modulus();

		//Set position and direction of ray
		dGeomRaySet(m_targetSensorRayGeom, rayPositionInWorldSpace[0], rayPositionInWorldSpace[1], rayPositionInWorldSpace[2], rayOrientation.Getx(), rayOrientation.Gety(), rayOrientation.Getz());

		//Set length of ray
		dGeomRaySetLength(m_targetSensorRayGeom, length);


		dContactGeom contact;

		if (dCollide(dGeomID(m_robotSpaceID), m_targetSensorRayGeom, 1, &contact, sizeof(dContactGeom)))
				return true;

		return false;


		//Test the ray for collisions against module geoms
		//dContactGeom contacts[2];

		//std::cout << "Ray position: " << rayPositionInWorldSpace[0] << " " << rayPositionInWorldSpace[1] << " " << rayPositionInWorldSpace[2] << std::endl;

		//std::cout << "Ray orientation: " << rayOrientation.Getx() << " " << rayOrientation.Gety() << " " << rayOrientation.Getz() << std::endl;

		//std::cout << "NUM MODULE GEOMS: " << m_moduleGeoms.size() << std::endl;

		//for (int i = 0; i != m_moduleGeoms.size(); i ++)
		//{

			//Test the ray for collisions against module geoms
			//dContactGeom contacts[1];

			/*if (dCollide(dGeomID(m_robotSpaceID), m_targetSensorRayGeom, 1, &contact, sizeof(dContactGeom)) > 1)
				return true;*/
		
			//if (dCollide(dGeomID(m_robotSpaceID), m_targetSensorRayGeom, 1, contacts, sizeof(dContactGeom)))
			//	return true;

			//if (dCollide(m_moduleGeoms[i], m_targetSensorRayGeom, 1, &contact, sizeof(dContactGeom)))
			//{
			//	//dVector3 result;
			//	//dBodyGetPosRelPoint(m_robotBodyID, contact.pos[0], contact.pos[1], contact.pos[2], result);
			//	//std::cout << "Contact point: " << result[0] << " " << result[1] << " " << result[2] << std::endl;

			//	//std::cout << "Geom 1: " << contact.g1 << std::endl;
			//	//std::cout << "Geom 2: " << contact.g2 << std::endl;

			//	float R[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};

			//	//dsDrawSphere(contact.pos, R, 0.1);

			//	//std::cout << "Can't see target" << std::endl << std::endl;
			//	returnValue = true;;
			//}

		//}
			//std::cout << "Can see target" << std::endl << std::endl;
		//return false;
	}

	else if (m_parameters->m_robotType == CUBIC)
	{




		dVector3 targetInBodyFrame;

		dBodyGetPosRelPoint(m_robotBodyID, targetPosition.Getx(), targetPosition.Gety(), targetPosition.Getz(), targetInBodyFrame);

		Vector3Df targetDelta = Vector3Df(targetInBodyFrame[0], targetInBodyFrame[1], targetInBodyFrame[2]) - m_position;

		targetDelta.Normalize();


		float angle = acos(targetDelta.DotWith(m_normal));


		//Check if goal is within sensor cone
		if (angle > m_targetDetectionThreshold)
		{
			return true;
		}


		if (angle < PI / float(2))
			return false;

		else
			return true;
	}

	return true;
}

bool Face::GetObstacleSensorInput()
{
	//Set the ray to the correct position, orientation and length. The position is the center of this face. The orientation is that such that it points at the target position, the length is that such it reaches the target position.

	//Position
	dVector3 rayPositionInWorldSpace;
	//dBodyGetRelPointPos(m_robotBodyID, m_position.Getx() + m_normal.Getx() /100, m_position.Gety() + m_normal.Gety() /100, m_position.Getz() + m_normal.Getz() /100, rayPositionInWorldSpace);

	dBodyGetRelPointPos(m_robotBodyID, m_position.Getx(), m_position.Gety(), m_position.Getz(), rayPositionInWorldSpace);


	//Orientation

	dVector3 rayOrientationInWorldSpace;
	dBodyVectorToWorld(m_robotBodyID, m_normal.Getx(), m_normal.Gety(), m_normal.Getz(), rayOrientationInWorldSpace);

	//Length
	float length = OBSTACLE_SENSOR_RANGE;

	//Set position and direction of ray
	dGeomRaySet(m_obstacleSensorRayGeom, rayPositionInWorldSpace[0], rayPositionInWorldSpace[1], rayPositionInWorldSpace[2], rayOrientationInWorldSpace[0], rayOrientationInWorldSpace[1], rayOrientationInWorldSpace[2]);

	//Set length of ray
	dGeomRaySetLength(m_obstacleSensorRayGeom, length);

	//Test the ray for collisions against obstacle geoms 
	//dContactGeom contact[2];

	dContactGeom contact;

	//int numCollisions = dCollide(dGeomID(m_obstacleSpaceID), m_obstacleSensorRayGeom, 2, contact, 1);

	int numCollisions = dCollide(dGeomID(m_obstacleSpaceID), m_obstacleSensorRayGeom, 1, &contact, 1);

	if (numCollisions)
		return true;

	else
		return false;
}

void Face::SetThrusterFiring(bool fire)
{
	if (m_thrusterFiring != fire)
		m_thrusterStateChanged = true;

	m_thrusterFiring = fire;
}

void Face::FireThruster()
{
	if (m_thrusterFiring && m_workingThruster)
	{
		Vector3Df force = -THRUST_FORCE * m_normal;

		dBodyAddRelForceAtRelPos(m_robotBodyID, force[0], force[1], force[2], m_position[0], m_position[1], m_position[2]);
	}
}

void Face::Step()
{	
}

bool Face::GetThrusterStateChanged() 
{
	if (m_thrusterStateChanged)
	{
		m_thrusterStateChanged = false;
		return true;
	}

	else
		return false;
}

void Face::RunTransPID(float error)
{
	m_PIDintegralTrans += error * CONTROL_STEP_TIME;

	float derivative = (error - m_PIDpreviousErrorTrans) / CONTROL_STEP_TIME;
	m_PIDoutputTrans = error + PID_TRANS_D_TO_TRANS_P_RATIO * derivative;
	m_PIDpreviousErrorTrans = error;
}

void Face::RunRotPID(float error)
{
	m_PIDintegralRot += error * CONTROL_STEP_TIME;

	float derivative = (error - m_PIDpreviousErrorRot) / CONTROL_STEP_TIME;
	m_PIDoutputRot = PID_ROT_P_TO_TRANS_P_RATIO * error + PID_ROT_D_TO_TRANS_P_RATIO * derivative;
	m_PIDpreviousErrorRot = error;
}

void Face::CollatePIDs()
{
	if (m_PIDoutputTrans + m_PIDoutputRot > 0)
		SetThrusterFiring(true);

	else
		SetThrusterFiring(false);
}