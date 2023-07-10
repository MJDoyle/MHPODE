#ifndef MODULE_HPP
#define MODULE_HPP

#include "Face.hpp"

class Module
{
	public:
		
		//Constructor
		Module(Vector3Df position, dMass mass, dSpaceID& space, dBodyID body, int pressureID, std::shared_ptr<Parameters> parameters);

		//Deconstructor
		~Module();

		//Get the position of the module
		Vector3Df GetPosition() {return m_position;}

		//Add a face to the module
		void AddFace(std::shared_ptr<Face> face) {m_faces.push_back(face);}

		//Draw the module
		void Draw();

		//Control step
		void Step(Vector3Df targetPosition);

		//Return the module geom
		dGeomID& GetGeom() {return m_geom;}

		//Return a list of all module faces
		std::vector<std::shared_ptr<Face>> GetFaces() {return m_faces;}

		//Return the number of thrusters that are firing during this time step
		int GetNumFiringThrusters();

	private:

		//A copy of the trial parameters
		std::shared_ptr<Parameters> m_parameters;

		//Body ID of robot
		dBodyID m_robotBodyID;

		//List of module faces
		std::vector<std::shared_ptr<Face>> m_faces;

		//This modules ODE geom ID
		dGeomID m_geom;

		//This modules position in the robot in robot coordinate frame
		Vector3Df m_position;
};


#endif