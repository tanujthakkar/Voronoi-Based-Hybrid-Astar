#ifndef VORONOI
#define VORONOI

#include "constants.h"
#include "helper.h"
#include "node2d.h"
#include "hybrid_astar.h"

extern std::map<uint, std::vector<uint>> voronoi_nodes; // Map of voronoi nodes with their respective neighbours

std::vector<std::vector<float>> voronoi_path();
float calc_node_cost(float x, float y, float gx, float gy, float cost_so_far);
float calc_yaw(float x_1, float y_1, float x_2, float y_2);
void voronoi_map();

#endif