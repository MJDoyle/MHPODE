#include "Config.hpp"



//SIMULATION VARIALBES

float PHYSICS_STEP_TIME = 0.1;
//float PHYSICS_STEP_TIME = 0.01;

float CONTROL_STEP_TIME = 1;

const int START_DELAY = 0;

//MODULE VARIABLES

float MODULE_SIZE = 0.08;			//0.1cm

float FLUID_DENSITY = 1000;					//1 g/cm3

float ROBOT_DENSITY = 1000;

float THRUST_FORCE = 0.0064;		// 0.1Dyne (0.000001N)

//0.0064 

//640 / 6400


float OBSTACLE_SENSOR_RANGE = 10;



//ENVIRONMENT VARIABLES


extern const Vector3Df STARTING_POSITION = Vector3Df(0, 0, 130);

const int DEBRIS_NUMBER = 12;

const float DEBRIS_MAX_SIZE = 5;

const float DEBRIS_MIN_SIZE = 8;
	





const float VICTORY_DISTANCE = 1;

const float MAX_DISTANCE = 20;

const float MAX_TRIAL_LENGTH = 25000;

const float FRICTION_CONSTANT = 0.5 * 0.8;

const float LINEAR_DRAG_CONSTANT = 100;


extern float PID_TRANS_D_TO_TRANS_P_RATIO = 1;
extern float PID_ROT_P_TO_TRANS_P_RATIO = 0;
extern float PID_ROT_D_TO_TRANS_P_RATIO = 0;