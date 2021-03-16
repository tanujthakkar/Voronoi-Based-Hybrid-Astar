#include "../include/hybrid_astar/dubins.h"

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

void dubins_node(Node4D* node, Node4D* g, float radius) {

	ob::StateSpacePtr space(new ompl::base::SE2StateSpace());

	ob::State* start = space->allocState();
	ob::State* goal = space->allocState();

	auto* s = start->as<ob::SE2StateSpace::StateType>();
	s->setX(node->get_x(node->get_size()-1));
	s->setY(node->get_y(node->get_size()-1));
	s->setYaw(node->get_yaw(node->get_size()-1));

	auto* t = goal->as<ob::SE2StateSpace::StateType>();
	t->setX(g->get_x(0));
	t->setY(g->get_y(0));
	t->setYaw(g->get_yaw(0));

	ob::DubinsStateSpace DP(radius, true);
	auto dubins_path = DP.dubins(start, goal);
	// cout << "Dubins Path: " << dubins_path << endl;
}