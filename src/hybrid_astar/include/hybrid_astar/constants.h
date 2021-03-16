#ifndef CONSTANTS
#define CONSTANTS

// Environment Constants
static const float XY_RESOLUTION = 0.05; // [m] Grid resolution of the map
static const float YAW_RESOLUTION = (15 * (M_PI / 180)); // [rad] Yaw resolution
static const float MOVE_STEP = 0.1; // [m] Path interpolate resolution
static const float PATH_LENGTH = 0.8; // [m] Length of the path create by each node
static const int N_LIST = ceil(PATH_LENGTH/MOVE_STEP);
static const float MIN_SAFE_DIST = 0.25; // [m] Minimum safe distance between vehicle and obstacles
static const int STEER_STEP = 3;

// Path accuracy
static const float XY_TOLERANCE = 0.8; // [m] Tolerance of error in goal position
static const float YAW_TOLERANCE = (10 * (M_PI / 180));

// Vehicle Configuration Constants
// Tractor/Robot Configuration
static const float WHEELBASE = 0.638; // [m] Wheelbase of the tractor, i.e., distance from front axle to rear axle
static const float RW = 0.793; // [m] Width of the tractor
static const float RL = 0.960; // [m] Length of the robot/tractor
static const float RF = 0.799; // [m] Distance from rear axle to front end
static const float RB = 0.161; // [m] Distance from rear axle to back end

// Trailer Configuration
static const float TW = 0.643; // [m] Width of the trailer
static const float TL = 1.0; // [m] Length of the trailer
static const float RTR = 0.4; // [m] Distance from the rear axle (hitch position) of the tractor to rear axle of the trailer
static const float RTF = 0.0; // [m] Distance from rear axle of tractor to trailer front end
static const float RTB = 1.0; // [m] Distance from rear axle of tractor to trailer back end

// Costs
static const float DIRECTION_CHANGE_COST = 100.0;
static const float BACKWARD_COST = 5.0;
static const float STEER_CHANGE_COST = 5.0;
static const float STEER_ANGLE_COST = 1.0;
static const float JACKNIFE_COST = 200.0;
static const float H_COST = 10.0;

static const int VECTOR_SIZE = ceil(PATH_LENGTH/MOVE_STEP); // Size of all the node vectors

#endif