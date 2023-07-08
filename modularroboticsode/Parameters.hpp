#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "Config.hpp"

//A simple container for all the parameters for a set of trials

//Some of these options are deprecated

struct Parameters
{
	Parameters()
	{}

	Parameters(int scenario, int controller, int trials, int robotType, int cbRtModules, bool randomOrientation, float thrusterForce, float moduleSize, float targetDistance, float PIDratio1, float PIDratio2, float PIDratio3, float thrusterMisalignment, float thrusterFailureRate, float sensorFailureRate, float sensorNoise, int startingSeed, float physicsStepTime, float controlStepTime, float obstacleSensorRange, float fluidDensity, float robotDensity, float fluidViscosity, int dragType)
	{
		m_scenario = scenario;	
		m_controller = controller;
		m_trials = trials;
		m_robotType = robotType;
		m_cbRtModules = cbRtModules;
		m_randomOrientation = randomOrientation;
		m_thrusterForce = thrusterForce;
		m_moduleSize = moduleSize;
		m_targetDistance = targetDistance;
		m_PIDRatio1 = PIDratio1;
		m_PIDRatio2 = PIDratio2;
		m_PIDRatio3 = PIDratio3;
		m_thrusterMisalignment = thrusterMisalignment;
		m_thrusterFailureRate = thrusterFailureRate;
		m_sensorFailureRate = sensorFailureRate;
		m_sensorNoise = sensorNoise;
		m_startingSeed = startingSeed;
		m_physicsStepTime = physicsStepTime;
		m_controlStepTime = controlStepTime;
		m_obstacleSensorRange = obstacleSensorRange;
		m_fluidDensity = fluidDensity;
		m_robotDensity = robotDensity;
		m_fluidViscosity = fluidViscosity;
		m_dragType = dragType;
	}

	int m_scenario;

	int m_controller;

	int m_trials;

	int m_robotType;

	int m_cbRtModules;

	bool m_randomOrientation;
	
	float m_thrusterForce;

	float m_moduleSize;

	float m_targetDistance;

	float m_PIDRatio1;

	float m_PIDRatio2;

	float m_PIDRatio3;

	float m_thrusterMisalignment;

	float m_thrusterFailureRate;

	float m_sensorFailureRate;

	float m_sensorNoise;

	int m_startingSeed;

	float m_physicsStepTime;

	float m_controlStepTime;

	float m_obstacleSensorRange;

	float m_fluidDensity;

	float m_robotDensity;

	float m_fluidViscosity;

	int m_dragType;


};

#endif