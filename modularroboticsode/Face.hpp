#ifndef FACE_HPP
#define FACE_HPP

#include "Parameters.hpp"

class Face
{
	public:

		//Constructor
		Face(Vector3Df position, Vector3Df normal, dMass mass, dSpaceID& robotSpace, dSpaceID& obstacleSpace, dBodyID& body, dGeomID& geom, std::vector<dGeomID> moduleGeoms, bool external, std::shared_ptr<Parameters> parameters);

		//Contructor without individual module geoms
		Face(Vector3Df position, Vector3Df normal, dMass mass, dSpaceID& robotSpace, dSpaceID& obstacleSpace, dBodyID& body, bool external, std::shared_ptr<Parameters> parameters);


		bool GetTargetSensorInput(Vector3Df targetPosition);

		bool GetObstacleSensorInput();

		Vector3Df GetPosition() {return m_position;}

		Vector3Df GetNormal() {return m_normal;}
		Vector3Df GetUnNormal() {return m_unNormal;}

		bool GetJoint() {return m_joint;}

		std::shared_ptr<Face> GetLinkedFace() {return m_linkedFace;}

		void SetLinkedface(std::shared_ptr<Face> face) {m_linkedFace = face;}

		void FireThruster();

		void Step();

		void Draw();

		bool GetExternal() {return m_external;}
		
		int GetFlowID() {return m_flowID;}
		void SetFlowID(int ID) {m_flowID = ID;}

		bool GetFlowChecked() {return m_flowChecked;}

		bool GetThrusterFiring() {return m_thrusterFiring;}

		void SetThrusterFiring(bool fire);

		bool GetThrusterStateChanged();

		void SetJoint(bool joint) {m_joint = joint;}

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

		//The vector along which the pump fires. This can be different from the actual normal if there is an error

		//The unnormalized normal
		Vector3Df m_unNormal;

		//Face direction
		int m_direction;

		//Position in robot coordinates
		Vector3Df m_position;

		//Vector3Df m_CoMrelativePosition;

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

		//Has this face been attached to another robot by a joint
		bool m_joint;



		//Has the flow been checked in the setting up process of the fluid routing
		bool m_flowChecked;

		//Has the thruster either stopped firing or started firing recently
		bool m_thrusterStateChanged;

		//Used for fluid routing


		float m_flowRate;

		int m_flowID;



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