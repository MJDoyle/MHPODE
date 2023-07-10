#include "Face.hpp"

//Constructor if module geoms are provided
Face::Face(Vector3Df position, Vector3Df normal, dMass mass, dSpaceID& robotSpace, dSpaceID& obstacleSpace, dBodyID& body, dGeomID& geom, std::vector<dGeomID> moduleGeoms, bool external, std::shared_ptr<Parameters> parameters) 
{
	m_parameters = parameters;
	m_position = position; 
	m_unNormal = normal;
	m_normal = 1 / MODULE_SIZE * normal; 

	m_robotSpaceID = robotSpace;
	m_obstacleSpaceID = obstacleSpace;
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

	m_moduleGeoms = moduleGeoms;

	m_robotBodyID = body;

	m_moduleGeomID = geom;

	m_targetSensorRayGeom = dCreateRay(0, 1000);

	m_obstacleSensorRayGeom = dCreateRay(0, OBSTACLE_SENSOR_RANGE);

	m_thrusterFiring = false;

	m_external = external;

	//Sensors not detecteding up to 90 degrees
	m_targetDetectionThreshold = float(90 - m_parameters->m_sensorNoise) * float(PI) / float(180);
}

//Contructor without individual module geoms
Face::Face(Vector3Df position, Vector3Df normal, dMass mass, dSpaceID& robotSpace, dSpaceID& obstacleSpace, dBodyID& body, bool external, std::shared_ptr<Parameters> parameters)
{
	m_parameters = parameters;
	m_position = position; 
	m_unNormal = normal;
	m_normal = 1 / MODULE_SIZE * normal; 

	m_robotSpaceID = robotSpace;
	m_obstacleSpaceID = obstacleSpace;
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

	m_robotBodyID = body;

	m_targetSensorRayGeom = dCreateRay(0, 1000);

	m_obstacleSensorRayGeom = dCreateRay(0, OBSTACLE_SENSOR_RANGE);

	m_thrusterFiring = false;

	m_external = external;

	//Sensors not detecteding up to 90 degress
	m_targetDetectionThreshold = float(90 - m_parameters->m_sensorNoise) * float(PI) / float(180);
}

//Draw the face. The colour changes depending on whether the thruster is firing during this frame
void Face::Draw()
{	
	if (m_thrusterFiring && m_workingThruster)
		dsSetColor (0, 0.3, 0);

	else
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

	dBodyGetRelPointPos(m_robotBodyID, m_position.Getx(), m_position.Gety(), m_position.Getz(), position);

	dsDrawBox (position, dBodyGetRotation(m_robotBodyID), S);
}

//Get the target sensor input. Case a ray from just above the face center to the target point. If the ray intersects with the robot then the target cannot be seen. For a cubic robot a simpler approach can be used
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

		//Orientation
		Vector3Df rayOrientation = targetPosition - Vector3Df(rayPositionInWorldSpace);

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
	}

	else if (m_parameters->m_robotType == CUBIC)
	{
		//Transform the target into the robot frame
		dVector3 targetInBodyFrame;

		dBodyGetPosRelPoint(m_robotBodyID, targetPosition.Getx(), targetPosition.Gety(), targetPosition.Getz(), targetInBodyFrame);

		//Get the delta vecotr between target and face
		Vector3Df targetDelta = Vector3Df(targetInBodyFrame[0], targetInBodyFrame[1], targetInBodyFrame[2]) - m_position;

		targetDelta.Normalize();

		//Calculate the angle between target and face normal
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

//Get the obstacle sensor input. Cast a ray from the face center along the face normal with a defined length. Retusn true if collisions with obstacles detected
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

	dContactGeom contact;

	//Get the collisiosn of the ray with the obstacle space
	int numCollisions = dCollide(dGeomID(m_obstacleSpaceID), m_obstacleSensorRayGeom, 1, &contact, 1);

	if (numCollisions)
		return true;

	else
		return false;
}

//Set the face thruster to fire or not, as determined by the controller
void Face::SetThrusterFiring(bool fire)
{
	m_thrusterFiring = fire;
}

//Fire thr thruster (add force to the robot) depending on whether it is set to fire, and whether it is working
void Face::FireThruster()
{
	if (m_thrusterFiring && m_workingThruster)
	{
		Vector3Df force = -THRUST_FORCE * m_normal;

		dBodyAddRelForceAtRelPos(m_robotBodyID, force[0], force[1], force[2], m_position[0], m_position[1], m_position[2]);
	}
}

//Update translational PID
void Face::RunTransPID(float error)
{
	m_PIDintegralTrans += error * CONTROL_STEP_TIME;

	float derivative = (error - m_PIDpreviousErrorTrans) / CONTROL_STEP_TIME;
	m_PIDoutputTrans = error + PID_TRANS_D_TO_TRANS_P_RATIO * derivative;
	m_PIDpreviousErrorTrans = error;
}

//Update rotational PID
void Face::RunRotPID(float error)
{
	m_PIDintegralRot += error * CONTROL_STEP_TIME;

	float derivative = (error - m_PIDpreviousErrorRot) / CONTROL_STEP_TIME;
	m_PIDoutputRot = PID_ROT_P_TO_TRANS_P_RATIO * error + PID_ROT_D_TO_TRANS_P_RATIO * derivative;
	m_PIDpreviousErrorRot = error;
}

//Collate the two PIDs to determine if the thruster should be set to fire
void Face::CollatePIDs()
{
	if (m_PIDoutputTrans + m_PIDoutputRot > 0)
		SetThrusterFiring(true);

	else
		SetThrusterFiring(false);
}