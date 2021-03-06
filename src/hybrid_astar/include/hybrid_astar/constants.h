#ifndef CONSTANTS
#define CONSTANTS

// Environment Constants
static const float XY_RESOLUTION = 0.05; // [m] Grid resolution of the map
static const float YAW_RESOLUTION = (15 * (M_PI / 180)); // [rad] Yaw resolution
static const float MOVE_STEP = 0.1; // [m] Path interpolate resolution
static const float MIN_SAFE_DIST = 0.3; // [m] Minimum safe distance between vehicle and obstacles

// Vehicle Configuration Constants
// Tractor/Robot Configuration
static const float PATH_LENGTH = 0.8; // [m] Length of the path create by each node
static const float WHEELBASE = 0.638; // [m] Wheelbase of the tractor, i.e., distance from front axle to rear axle
static const float WIDTH = 0.793; // [m] Width of the tractor
static const float RF = 0.799; // [m] Distance from rear axle to front end
static const float RB = 0.161; // [m] Distance from rear axle to back end

// Trailer Configuration
static const float RTR = 0.8; // [m] Distance from the rear axle (hitch position) of the tractor to rear axle of the trailer
static const float RTF = 0.2; // [m] Distance from rear axle of tractor to trailer front end
static const float RTB = 0.6; // [m] Distance from rear axle of tractor to trailer back end

// Costs
static const float DIRECTION_CHANGE_COST = 100.0;
static const float BACKWARD_COST = 5.0;
static const float STEER_CHANGE_COST = 5.0;
static const float STEER_ANGLE_COST = 1.0;
static const float H_COST = 10.0;

#endif