#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <breadcrumb/RequestPath.h>
#include <breadcrumb/AStarParamsConfig.h>

#include <breadcrumb/AStar.h>

#include <string>

class Breadcrumb {
	private:
		ros::NodeHandle nhp_;

		ros::Subscriber sub_grid_;
		ros::ServiceServer srv_request_path_;

		dynamic_reconfigure::Server<breadcrumb::AStarParamsConfig> dyncfg_settings_;

		std::string topic_input_grid_;
		std::string topic_output_array_;

		nav_msgs::MapMetaData map_info_;
		std::string frame_id_;
		AStar::Generator astar_;

		bool flag_got_grid_;
		int param_obstacle_threshold_;
		bool param_calc_sparse_;
		bool param_theta_star_;

		bool enforce_bounds_ = false;
		double x_min_ = -4, x_max_ = 4;
		double y_min_ = -2.75, y_max_ = 2.25;
		double z_min_ = 0.5,  z_max_ = 4.0;


	public:
		Breadcrumb( void );

		~Breadcrumb( void );

	private:
		void callback_cfg_settings( breadcrumb::AStarParamsConfig &config, uint32_t level );

		void callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg_in);

		bool request_path(breadcrumb::RequestPath::Request& req, breadcrumb::RequestPath::Response& res);

		// simple clamp
		inline double clamp_(double v, double lo, double hi) const {
		    return std::max(lo, std::min(v, hi));
		}

		// convert a grid index to world center
		inline void cellToWorld_(int i, int j, double& wx, double& wy) const {
		    wx = (i + 0.5) * map_info_.resolution + map_info_.origin.position.x;
		    wy = (j + 0.5) * map_info_.resolution + map_info_.origin.position.y;
		}

		// ring-search for nearest free cell (i,j) inside masked grid
		bool snapToNearestFree_(int& i, int& j) const;
};
