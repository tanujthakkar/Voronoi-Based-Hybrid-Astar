#ifndef CONSTANTS
#define CONSTANTS

#include<cmath>

// Environment Constants
static const float XY_RESOLUTION = 0.05; // [m] Grid resolution of the map
static const float YAW_RESOLUTION = (15 * (M_PI / 180)); // [rad] Yaw resolution
static const float MOVE_STEP = 0.1; // [m] Path resolution
static const float PATH_LENGTH = 0.8; // [m] Length of the path create by each node
static const int VECTOR_SIZE = ceil(PATH_LENGTH/MOVE_STEP); // Size of node vectors
static const float MIN_SAFE_DIST = 0.10; // [m] Minimum safe distance between vehicle and obstacles
static const int STEER_STEP = 16; // Number of steering inputs (x2 total)

// Path accuracy
static const float XY_TOLERANCE = 0.8; // [m] Tolerance of error in solution
static const float YAW_TOLERANCE = (5 * (M_PI / 180)); // [rad] Tolerance of yaw in solution

// Vehicle Configuration Constants
// Tractor/Robot Configuration
static const float WHEELBASE = 0.638; // [m] Wheelbase of the tractor, i.e., distance from front axle to rear axle
static const float RW = 0.793; // [m] Width of the tractor
static const float RL = 0.960; // [m] Length of the robot/tractor
static const float RF = 0.799; // [m] Distance from rear axle to front end
static const float RB = 0.161; // [m] Distance from rear axle to back end
static const float DELTAR = (RF - RB) / 2.0; // [m] Half of Wheelbase

// Trailer Configuration
static const float TW = 0.643; // [m] Width of the trailer
static const float TL = 1.0; // [m] Length of the trailer
static const float RTR = 0.9; // [m] Distance from the rear axle (hitch position) of the tractor to rear axle of the trailer
static const float RTF = 0.025; // [m] Distance from rear axle of tractor to trailer front end
static const float RTB = 0.975; // [m] Distance from rear axle of tractor to trailer back end
static const float DELTAT = (RTF - RTB) / 2.0; // [m] Half the distance from hitch point to rear-axle

// Costs
static const float DIRECTION_CHANGE_COST = 100.0; // Penalizing change in direction of motion
static const float BACKWARD_COST = 5.0; // Penalizing backwards motion
static const float STEER_CHANGE_COST = 5.0; // Penalizing change in steering input
static const float STEER_ANGLE_COST = 1.0; // Penalizing steering input
static const float JACKNIFE_COST = 200.0; // Penalizing high hitch-angle
static const float H_COST = 10.0; // Heuristic cost weight

#endif