#include "Obstacle.hpp"

//Constructor
Obstacle::Obstacle(Vector3Df size, Vector3Df position, dWorldID& worldID, dSpaceID& spaceID, bool kinematic)
{
	m_size = size;

	m_bodyID = dBodyCreate(worldID);

	dMass totalMass;
	dMassSetZero (&totalMass);

	//If the x and y sizes are 0, then create a sphere with radius defined by the z size
	if (size.Getx() == 0 && size.Gety() == 0)
		dMassSetSphere(&totalMass, ROBOT_DENSITY * 100, size.Getz());

	//Ohterwise create a box
	else
		dMassSetBox(&totalMass, ROBOT_DENSITY * 100, size.Getx(), size.Gety(), size.Getz());

	//Set the mass of the object
	dBodySetMass (m_bodyID, &totalMass);

	//Set whether the body will be affected by forces and collisions
	if (kinematic)
		dBodySetKinematic(m_bodyID);

	//Create a sphere geometry
	if (size.Getx() == 0 && size.Gety() == 0)
		m_geomID = dCreateSphere(spaceID, size.Getz());

	//Create a box geometry
	else
		m_geomID = dCreateBox(spaceID, size.Getx(), size.Gety(), size.Getz());


	//Set the geom and position
	dGeomSetBody(m_geomID, m_bodyID);
	dBodySetPosition(m_bodyID, position.Getx(), position.Gety(), position.Getz());
}

//Deconstructor - destroy the body and geom
Obstacle::~Obstacle()
{
	dGeomDestroy(m_geomID);
	dBodyDestroy(m_bodyID);
}

//Get the object position in world space as a Vector3Df
Vector3Df Obstacle::GetPosition()
{
	const float* pos = dBodyGetPosition(m_bodyID);

	return Vector3Df(pos[0], pos[1], pos[2]);
}

//Draw the obstacle
void Obstacle::Draw()
{
	dsSetColor (0.3, 0.3, 0.3);

	const dVector3 S = {m_size.Getx(), m_size.Gety(), m_size.Getz()};

	if (S[0] == 0 && S[1] == 0)
		dsDrawSphere(dGeomGetPosition(m_geomID), dGeomGetRotation(m_geomID), S[2]);

	else
		dsDrawBox (dGeomGetPosition(m_geomID), dGeomGetRotation(m_geomID), S);
}