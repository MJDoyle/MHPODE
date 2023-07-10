#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include "Robot.hpp"

class Obstacle
{
	public:

		//Constructor
		Obstacle(Vector3Df size, Vector3Df position, dWorldID& worldID, dSpaceID& spaceID, bool kinematic);

		//Destructor
		~Obstacle();

		//Draw the obstacle
		void Draw();

		//Get the obstacle position in world space
		Vector3Df GetPosition();

	private:

		//Original position of the object
		Vector3Df m_startingPosition;

		//Body ID for ODE
		dBodyID m_bodyID;

		//Geom ID for ODE
		dGeomID m_geomID;

		//Object size
		Vector3Df m_size;

};

#endif