#ifndef HELPER
#define HELPER

#include <cmath>
#include <geometry_msgs/PolygonStamped.h>

// Helper function to normalize heading to [0, 360]
static inline float normalize_heading_rad(float t) {
	if (t < 0.0) {
		t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
		return 2.f * M_PI + t;
	}

	return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

// Helper function to normalize heading to [-3.14, 3.14]
static inline float pi_2_pi(float h) {

	while(h > M_PI) {
		h -= 2.0 * M_PI;
	}

	while(h < -M_PI) {
		h += 2.0 * M_PI;
	}

	return h;
}

// Helper function to convert radians to degree
static inline float to_deg(float t) {
	return (t * 180.f / M_PI);
}

// Helper function to convert degrees to radians
static inline float to_rad(float t) {
	return (t / 180.f * M_PI);
}

static inline float euclidean_2d(float x0, float y0, float x1, float y1) {
	return (hypot(x0 - x1, y0 - y1));
}

#endif