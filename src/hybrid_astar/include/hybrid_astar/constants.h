#ifndef CONSTANTS
#define CONSTANTS

// Environment Constants
static const float XY_RESOLUTION = 0.05; // [m] Grid resolution of the map
static const float YAW_RESOLUTION = (15 * (M_PI / 180)); // [rad] Yaw resolution
static const float MOVE_STEP = 0.1; // [m] Path interpolate resolution

// Vehicle Configuration Constants
static const float WHEELBASE = 0.8; // [m] Wheelbase of the tractor, i.e., distance from front axle to rear axle
static const float RTR = 0.8; // [m] Distance from the rear axle (hitch position) of the tractor to rear axle of the trailer

// Costs
static const float DIRECTION_CHANGE_COST = 100.0;
static const float BACKWARD_COST = 5.0;
static const float STEER_CHANGE_COST = 5.0;
static const float STEER_ANGLE_COST = 1.0;
static const float H_COST = 10.0;

#endif