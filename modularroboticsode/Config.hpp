#ifndef CONFIG_HPP
#define CONFIG_HPP

#define dSINGLE

//#include "Eigen/Dense"

#include "MiscFunctions.h"
#include "VectorFunctions.h"

#include <drawstuff/drawstuff.h>

#include "resource.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "Textures"
#endif

#include <vector>
#include <map>
#include <list>
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>

#include <cstdio>
#include <ctime>

#include <cassert>

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <random>
#include <tuple>


#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#define PI 3.14159265

#define e 2.71828182

#define MAX_CONTACTS 8



//SIMULATION VARIALBES

extern float PHYSICS_STEP_TIME;

extern float CONTROL_STEP_TIME;

extern const int START_DELAY;

//MODULE VARIABLES

extern float MODULE_SIZE;

extern float FLUID_DENSITY;

extern float ROBOT_DENSITY;

extern float THRUST_FORCE;

extern float OBSTACLE_SENSOR_RANGE;

//ENVIRONMENT VARIABLES

extern const Vector3Df STARTING_POSITION;

extern const int DEBRIS_NUMBER;

extern const float DEBRIS_MAX_SIZE;

extern const float DEBRIS_MIN_SIZE;

//Enum for different scenario types
const enum SCENARIO {OPEN, OBSTACLES};

//Enum for different robot types
const enum ROBOT {CUBIC, RANDOM, CONVEX};

//Enum for different controller types
const enum CONTROLLER {DECENTRALIZED_BASIC, DECENTRALIZED_COMM, DECENTRALIZED_AVOID, CENTRALIZED};

//Enum for differnt drag types
const enum DRAG {LINEAR, QUADRATIC};

extern const float VICTORY_DISTANCE;

extern const float MAX_DISTANCE;

extern const float MAX_TRIAL_LENGTH;

extern const float FRICTION_CONSTANT;

extern const float LINEAR_DRAG_CONSTANT;

const Vector3Df directions[6] = {Vector3Df(1, 0, 0), Vector3Df(-1, 0, 0), Vector3Df(0, 1, 0), Vector3Df(0, -1, 0), Vector3Df(0, 0, 1), Vector3Df(0, 0, -1)};

//PID values for centralised controller

extern float PID_TRANS_D_TO_TRANS_P_RATIO;
extern float PID_ROT_P_TO_TRANS_P_RATIO;
extern float PID_ROT_D_TO_TRANS_P_RATIO;


#endif