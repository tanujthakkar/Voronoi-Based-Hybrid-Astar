#include "../include/hybrid_astar/hybrid_astar_planner.h"
#include "hybrid_astar/GlobalPath.h">

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace hybrid_astar_planner {

	HybridAStarPlanner::HybridAStarPlanner(){
	}

	HybridAStarPlanner::HybridAStarPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros) {
		initialize(name, costmap_ros);
	}

	void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
		sc = nh.serviceClient<hybrid_astar::GlobalPath>("hybrid_astar_planner_service");
		// global_plan_pub = nh.advertise<nav_msgs::Path>("global_planner", 10);
		sc.waitForExistence();
	}

	bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
		
		hybrid_astar::GlobalPath p;
		geometry_msgs::PoseStamped pose_stamped;

		sc.call(p);

		// path.header.stamp = ros::Time::now();
		// path.header.frame_id = "/map";
		// path.poses.clear();

		for (int i = 0; i < p.response.plan.poses.size(); ++i) {
			pose_stamped.header.stamp = ros::Time::now();
			pose_stamped.header.frame_id = "map";
			pose_stamped.pose = p.response.plan.poses[i].pose;
			// path.poses.push_back(pose_stamped);
			plan.push_back(pose_stamped);
		}

		// global_plan_pub.publish(path);

		return true;
	}
}