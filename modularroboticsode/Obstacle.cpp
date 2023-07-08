#include "Obstacle.hpp"

Obstacle::Obstacle(Vector3Df size, Vector3Df position, dWorldID& worldID, dSpaceID& spaceID, bool kinematic)
{
	m_size = size;

	m_bodyID = dBodyCreate(worldID);

	dMass totalMass;
	dMassSetZero (&totalMass);

	if (size.Getx() == 0 && size.Gety() == 0)
		dMassSetSphere(&totalMass, ROBOT_DENSITY * 100, size.Getz());

	else
		dMassSetBox(&totalMass, ROBOT_DENSITY * 100, size.Getx(), size.Gety(), size.Getz());

	dBodySetMass (m_bodyID, &totalMass);

	if (kinematic)
		dBodySetKinematic(m_bodyID);



	if (size.Getx() == 0 && size.Gety() == 0)
		m_geomID = dCreateSphere(spaceID, size.Getz());

	else
		m_geomID = dCreateBox(spaceID, size.Getx(), size.Gety(), size.Getz());



	dGeomSetBody(m_geomID, m_bodyID);
	dBodySetPosition(m_bodyID, position.Getx(), position.Gety(), position.Getz());
}


Obstacle::~Obstacle()
{
	dGeomDestroy(m_geomID);
	dBodyDestroy(m_bodyID);
}

Vector3Df Obstacle::GetPosition()
{
	const float* pos = dBodyGetPosition(m_bodyID);

	return Vector3Df(pos[0], pos[1], pos[2]);
}

void Obstacle::Draw()
{
	dsSetColor (0.3, 0.3, 0.3);

	const dVector3 S = {m_size.Getx(), m_size.Gety(), m_size.Getz()};

	if (S[0] == 0 && S[1] == 0)
		dsDrawSphere(dGeomGetPosition(m_geomID), dGeomGetRotation(m_geomID), S[2]);

	else
		dsDrawBox (dGeomGetPosition(m_geomID), dGeomGetRotation(m_geomID), S);
}