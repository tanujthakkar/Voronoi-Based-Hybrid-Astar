#ifndef NODE2D
#define NODE2D

#include "constants.h"
#include "helper.h"
#include "hybrid_astar.h"

using namespace std;

class Node2D {

public:

	Node2D() {};

	Node2D(float x, float y, float cost, uint pind) {

		this->x = x;
		this->y = y;
		this->cost = cost;
		this->g_cost = 0;
		this->pind = pind;
	}

	float get_x() const { return x; }

	float get_y() const { return y; }

	float get_cost() const { return cost; }

	float get_g_cost() const { return g_cost; }

	uint get_pind() const { return pind; }

	void set_g_cost(float g_cost) { this->g_cost = g_cost; }

private:

	float x; // x co-ordinate of the node
	float y; // y co-ordinate of the node
	float cost; // cost of the node
	float g_cost; // g cost of the node
	uint pind; // parent index of the node

};

#endif