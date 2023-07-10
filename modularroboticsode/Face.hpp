#ifndef FACE_HPP
#define FACE_HPP

#include "Parameters.hpp"

class Face
{
	public:

		//Constructor if module geoms are provided
		Face(Vector3Df position, Vector3Df normal, dMass mass, dSpaceID& robotSpace, dSpaceID& obstacleSpace, dBodyID& body, dGeomID& geom, std::vector<dGeomID> moduleGeoms, bool external, std::shared_ptr<Parameters> parameters);

		//Contructor without individual module geoms
		Face(Vector3Df position, Vector3Df normal, dMass mass, dSpaceID& robotSpace, dSpaceID& obstacleSpace, dBodyID& body, bool external, std::shared_ptr<Parameters> parameters);

		//Check the target sensor for this face. Returns true if target is occluded
		bool GetTargetSensorInput(Vector3Df targetPosition);

		//Check the obstacle sensor for this face. Returns true if obstacle is detected
		bool GetObstacleSensorInput();

		//Get various properties of the face
		Vector3Df GetPosition() {return m_position;}
		Vector3Df GetNormal() {return m_normal;}
		Vector3Df GetUnNormal() {return m_unNormal;}

		//Get the face linked to theis face, for use with communication
		std::shared_ptr<Face> GetLinkedFace() {return m_linkedFace;}

		//Set the linked face
		void SetLinkedface(std::shared_ptr<Face> face) {m_linkedFace = face;}

		//Fire the thruster if it has been set to fire and is working
		void FireThruster();

		//Draw the face
		void Draw();

		//Is this face external?
		bool GetExternal() {return m_external;}
		
		//Is the thruster firing during the current step?
		bool GetThrusterFiring() {return m_thrusterFiring;}

		//Set the thruster to fire or not
		void SetThrusterFiring(bool fire);

		//Run the PIDs for the centralized controller
		void RunTransPID(float error);

		void RunRotPID(float error);

		void CollatePIDs();

	private:

		//Parameters
		std::shared_ptr<Parameters> m_parameters;

		//Outward normal to surface in robot coordinates
		Vector3Df m_normal;

		//Linked face for communication
		std::shared_ptr<Face> m_linkedFace;

		//The unnormalized normal
		Vector3Df m_unNormal;

		//Face direction
		int m_direction;

		//Position in robot coordinates
		Vector3Df m_position;

		//Does the face have a target sensor
		bool m_targetSensor;

		//Ray geom for target sensor
		dGeomID m_targetSensorRayGeom;

		//Ray geom for obstacle sensor
		dGeomID m_obstacleSensorRayGeom;

		//Body ID of robot
		dBodyID m_robotBodyID;

		//Robot space
		dSpaceID m_robotSpaceID;

		//Obstacle space
		dSpaceID m_obstacleSpaceID;

		//Module geom
		dGeomID m_moduleGeomID;

		//Vector of all module geoms for ray casting
		std::vector<dGeomID> m_moduleGeoms;

		//Is the thruster firing
		bool m_thrusterFiring;

		//Is the face external (on the robot - environment boundary)
		bool m_external;

		//Faults
		bool m_workingThruster;

		//Threshold for target detection (90 by default)
		float m_targetDetectionThreshold;

		bool m_faultyTargetSensor;

		bool m_faultyTargetSensorState;

		bool m_faultyRangeSensor;

		bool m_faultyRangeSensorState;


		//Translation PID variables
		float m_PIDpreviousErrorTrans;

		float m_PIDoutputTrans;

		float m_PIDintegralTrans;


		//Rotation PID variables
		float m_PIDpreviousErrorRot;

		float m_PIDoutputRot;

		float m_PIDintegralRot;
};

#endif