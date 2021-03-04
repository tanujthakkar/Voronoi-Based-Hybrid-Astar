#ifndef HELPER
#define HELPER

#include <cmath>

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

// Helper function to calculate path length
static inline float path_length(std::vector<float> x, std::vector<float> y) {
	float cost = 0.0;

	for (int i = 1; i < x.capacity(); ++i)
	{
		cost = cost + sqrt(pow(x[i] - x[i-1], 2) + pow(y[i] - y[i-1], 2) * 1.0);
	}

	return cost;
}

#endif