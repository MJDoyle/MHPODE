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

		Vector3Df GetPosition() {return m_position;}

		Vector3Df CoMRelativePosition() {return m_CoMrelativePosition;}

		void AddFace(std::shared_ptr<Face> face) {m_faces.push_back(face);}

		void Draw();

		void Step(Vector3Df targetPosition);

		dGeomID& GetGeom() {return m_geom;}

		std::vector<std::shared_ptr<Face>> GetFaces() {return m_faces;}

		std::shared_ptr<Face> GetOppositeFace(Vector3Df unNormal);

		int GetNumFiringThrusters();

	private:

		//Parameters
		std::shared_ptr<Parameters> m_parameters;

		//Body ID of robot
		dBodyID m_robotBodyID;

		std::vector<std::shared_ptr<Face>> m_faces;

		//This modules ODE geom ID
		dGeomID m_geom;

		//This modules position in the robot in robot coordinate frame
		Vector3Df m_position;

		Vector3Df m_CoMrelativePosition;
};


#endif