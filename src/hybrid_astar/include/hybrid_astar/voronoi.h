#ifndef VORONOI
#define VORONOI

#include "constants.h"
#include "helper.h"
#include "node2d.h"
#include "hybrid_astar.h"

extern std::map<uint, std::vector<uint>> voronoi_nodes; // Map of voronoi nodes with their respective neighbours

std::vector<std::vector<float>> voronoi_path();
void voronoi_map();

#endif