#ifndef WORLD_HPP
#define WORLD_HPP

#include "Obstacle.hpp"

class Population;

class World
{
	public:

		//Contructor
		World(std::vector<std::shared_ptr<Parameters>> parameters, bool graphicsEnabled);	

		//Reset the world - used at the end of each trial to prepare for the next one
		void Reset();

		//Basic update function - calls the ODE physics update and the world Step function
		void Update();

		//Updates robots, running controllers
		void Step();

		//Set up a robot for upcoming trial according to current parameters
		void SetupRobot();

		//Set up obstacles for upcoming trial according to current parameters
		void SetupObstacles();

		//Various obstacle-adding functions. Some deprecated
		void AddNewObstacle(Vector3Df size, Vector3Df position);

		void GenerateRandomDebris();

		void GenerateSlalomObstacles();

		MatrixNMf GenerateStartingOrientation();

		//Define objects for drawstuff to draw
		void Draw();

	private:

		//Parameters
		std::vector<std::shared_ptr<Parameters>> m_parameters;

		//Iterator to current set of parameters
		std::vector<std::shared_ptr<Parameters>>::iterator m_currentParameters;

		//Index of current set of parameters
		int m_parametersCounter;

		//Current trial number in current set of parameters
		int m_trialCounter;

		//Trial length in simulation seconds
		float m_trialLength;

		std::clock_t m_startTime;

		float m_controlStepCounter;

		//Files for outputting trial data to
		std::vector<std::ofstream> m_trialOutput;

		std::ofstream m_positionOutput;

		//World ID for ODE
		dWorldID m_worldID;

		//Collisions space ID for ODE (robots)
		dSpaceID m_spaceID;

		//Collisions space ID for obstacles
		dSpaceID m_obstacleSpaceID;

		//Contact joint group
		dJointGroupID m_contacts;

		//The robots in the simulation
		std::vector<std::shared_ptr<Robot>> m_robots;

		//Obstacles in the simulation
		std::vector<std::shared_ptr<Obstacle>> m_obstacles;

		//The target position
		Vector3Df m_targetPosition;	

		//Counter for delay at the start of the program (for recording video etc.)
		int m_delayCounter;

		//Takes a module map and checks that's its correct (returns the correct version)
		std::vector<Vector3Df> ConvertMap(std::vector<Vector3Df> map);

		//Handle collisions between robots and obstacles, and in some cases between obstacles and other obstacles
		void HandleCollisions();

		//Check if the robot(s) has succesfully completed the trial
		bool CheckVictoryConditions();

		//Will graphical output be produced
		bool m_graphicsEnabled;

		//Set target position for upcoming trial
		void SetupTargetPosition();

		//Deprecated function for robot magnet interaction
		void InteractRobots();

		//Deprecated function for merging docking robots together
		void MergeRobots(std::shared_ptr<Robot> robot1, std::shared_ptr<Robot> robot2, std::weak_ptr<Face> face1, std::weak_ptr<Face> face2);
};

#endif