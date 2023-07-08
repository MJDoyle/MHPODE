#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "Module.hpp"

class Robot
{
	public:

		//Constructor
		Robot(dWorldID& worldID, dSpaceID& robotSpaceID, dSpaceID& obstacleSpaceID, std::vector<Vector3Df> modules, std::vector<Vector3Di> unscaledModules, Vector3Df startingPosition, MatrixNMf startingOrientation, std::shared_ptr<Parameters> parameters);

		//Deconstructor (must remove all ODE bodies etc.)
		~Robot();

		void Draw();

		void ControlStep(Vector3Df targetPosition);

		void ApplyThrusterForces();

		void ApplyDrag();

		Vector3Df GetPosition();
		std::vector<std::weak_ptr<Face>> GetFaces() {return m_faces;}
		dBodyID GetBodyID() {return m_bodyID;}
		long long GetThrusterFiringCounter() {return m_thrusterFiringCounter;}

		dSpaceID GetRobotSpace() {return m_robotSpaceID;}

		void CalculatePseudoInverse();

		void IncrementCollisionCounter() {m_collisionCounter ++;}

		long long GetCollisionCounter() {return m_collisionCounter;}

		std::vector<float> GetAABB() {return m_AABB;}

private:

		//Parameters
		std::shared_ptr<Parameters> m_parameters;

		//All the modules of the robot
		std::vector<std::shared_ptr<Module>> m_modules;

		//Weak pointers to all the faces of the robot (mainly for drag calculation purposes)
		std::vector<std::weak_ptr<Face>> m_faces;

		//Body ID for ODE
		dBodyID m_bodyID;

		dSpaceID m_robotSpaceID;

		void SetStartingOrientation();
		
		void SetLinkedFaces();

		//Total number of external thrusters
		int m_numExternalThrusters;

		//Counter for pump firing 
		long long m_thrusterFiringCounter;

		//Counter for collisions with obstacles
		long long m_collisionCounter;

		//////////////
		//
		//	CENTRALIZED CONTROLLER
		//
		//////////////

		MatrixNMf m_transPseudoInverse;

		MatrixNMf m_rotPseudoInverse;

		//Map of the modules, used for flow routing purposes
		std::map<Vector3Di, std::shared_ptr<Module>> m_moduleMap; //In module coordinates

		std::vector<float> m_AABB;
};


#endif