#include <ros/ros.h>

#include <breadcrumb/breadcrumb.h>
#include <breadcrumb/RequestPath.h>
#include <breadcrumb/AStarParamsConfig.h>

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <math.h>

Breadcrumb::Breadcrumb() :
	nhp_("~"),
	flag_got_grid_(false),
	param_calc_sparse_(false),
	dyncfg_settings_(nhp_) {

	dyncfg_settings_.setCallback(boost::bind(&Breadcrumb::callback_cfg_settings, this, _1, _2));

	// Bounds (mission limits)
	nhp_.param("enforce_bounds", enforce_bounds_, true);
	nhp_.param("x_min", x_min_, x_min_);
	nhp_.param("x_max", x_max_, x_max_);
	nhp_.param("y_min", y_min_, y_min_);
	nhp_.param("y_max", y_max_, y_max_);
	nhp_.param("z_min", z_min_, z_min_);
	nhp_.param("z_max", z_max_, z_max_);

	sub_grid_ = nhp_.subscribe<nav_msgs::OccupancyGrid>( "grid", 10, &Breadcrumb::callback_grid, this );

	ROS_INFO("[Breadcrumb] Waiting for occupancy grid");
}

Breadcrumb::~Breadcrumb() {
}

void Breadcrumb::callback_cfg_settings( breadcrumb::AStarParamsConfig &config, uint32_t level ) {
	param_obstacle_threshold_ = config.obstacle_threshold;
	param_calc_sparse_ = config.calc_sparse_path;
	param_theta_star_ = config.any_angle;

	astar_.setThetaStar(param_theta_star_);
	astar_.setDiagonalMovement(config.allow_diagonals);

	switch(config.search_heuristic) {
		case 0: {
			astar_.setHeuristic(AStar::Heuristic::manhattan);
			break;
		}
		case 1: {
			astar_.setHeuristic(AStar::Heuristic::euclidean);
			break;
		}
		case 2: {
			astar_.setHeuristic(AStar::Heuristic::octagonal);
			break;
		}
		default: {
			ROS_ERROR("[Breadcrumb] Error setting heuristic, reverting to 'Manhattan'");
			astar_.setHeuristic(AStar::Heuristic::manhattan);
		}
	};
}

bool Breadcrumb::request_path(breadcrumb::RequestPath::Request& req, breadcrumb::RequestPath::Response& res) {
	res.path.header.frame_id = frame_id_;
	res.path.header.stamp = ros::Time::now();

	if( param_calc_sparse_ )
		res.path_sparse.header = res.path.header;

	if (enforce_bounds_) {
		req.start.x = clamp_(req.start.x, x_min_, x_max_);
		req.start.y = clamp_(req.start.y, y_min_, y_max_);
		req.end.x   = clamp_(req.end.x,   x_min_, x_max_);
		req.end.y   = clamp_(req.end.y,   y_min_, y_max_);
	}

	int start_i = (int)( (req.start.x - map_info_.origin.position.x) / map_info_.resolution);
	int start_j = (int)( (req.start.y - map_info_.origin.position.y) / map_info_.resolution);
	int end_i = (int)( (req.end.x - map_info_.origin.position.x) / map_info_.resolution);
	int end_j = (int)( (req.end.y - map_info_.origin.position.y) / map_info_.resolution);

	// If start/end are in collision (or outside masked box), snap to nearest free cell
	int si = start_i, sj = start_j, ei = end_i, ej = end_j;
	
	if (astar_.detectCollision({si, sj})) {
		if (!snapToNearestFree_(si, sj)) {
			ROS_ERROR("[Breadcrumb] Requested start is within an obstacle and no nearby free cell inside bounds");
			return true; // keep service reply semantics
		}
		// update clamped world too (optional)
		cellToWorld_(si, sj, req.start.x, req.start.y);
	}
	
	if (astar_.detectCollision({ei, ej})) {
		if (!snapToNearestFree_(ei, ej)) {
			ROS_ERROR("[Breadcrumb] Requested end is within an obstacle and no nearby free cell inside bounds");
			return true;
		}
		cellToWorld_(ei, ej, req.end.x, req.end.y);
	}
	
	// overwrite the i/j we pass into A*
	start_i = si; start_j = sj;
	end_i   = ei; end_j   = ej;

	ROS_DEBUG("[Breadcrumb] Start/End: [%i, %i]; [%i, %i]", start_i, start_j, end_i, end_j);

	std::vector<AStar::Vec2i> path;

	if( ( start_i < 0 ) || ( start_i >= map_info_.width ) ||
		( start_j < 0 ) || ( start_j >= map_info_.height ) ||
		( end_i < 0 ) || ( end_i >= map_info_.width ) ||
		( end_j < 0 ) || ( end_j >= map_info_.height ) ) {
		ROS_ERROR("[Breadcrumb] Requested start/end out of bounds");

		return true;
	}

	if(astar_.detectCollision({end_i, end_j})) {
		ROS_ERROR("[Breadcrumb] Requested end is within a obstacle");
		return true;
	}

	path = astar_.findPath({start_i, start_j}, {end_i, end_j});

	if(path.size() > 1) {
		if( ( path[path.size() - 1].x == start_i ) &&
			( path[path.size() - 1].y == start_j ) &&
			( path[0].x == end_i ) &&
			( path[0].y == end_j ) ) {

			ROS_DEBUG("[Breadcrumb] Solution found!");

			int sk_last = path.size()-1;

			for(int k=path.size()-1; k>=0; k--) {
				geometry_msgs::Pose step;

				//Calculate position in the parent frame
				step.position.x = (path[k].x * map_info_.resolution) + (map_info_.resolution / 2) + map_info_.origin.position.x;
				step.position.y = (path[k].y * map_info_.resolution) + (map_info_.resolution / 2) + map_info_.origin.position.y;
				const double z_path = enforce_bounds_ ? clamp_(req.start.z, z_min_, z_max_) : req.start.z;
				step.position.z = z_path;

				//Fill in the rotation data
				if(k > 0) {
					double yaw = atan2(path[k-1].y - path[k].y, path[k-1].x - path[k].x);

					tf2::Quaternion q;
					q.setEuler(0.0, 0.0, yaw);

					step.orientation.w = q.getW();
					step.orientation.x = q.getX();
					step.orientation.y = q.getY();
					step.orientation.z = q.getZ();

				} else {
					//This is the last value in the set, use the same orientation as the second last value
					step.orientation = res.path.poses.back().orientation;
				}

				res.path.poses.push_back(step);

				ROS_DEBUG("[Breadcrumb] Path: %d, %d", path[k].x, path[k].y);

				//Only calculate if we should, and we're testing a final pass
				if( param_calc_sparse_ && !param_theta_star_ ) {
					if( (k > 0) && (k < sk_last) ) {
						//Calculate the angle from the last sparse step (sk_last) to k.
						double dx = path[k].x - path[sk_last].x;
						double dy = path[k].y - path[sk_last].y;
						double sang = std::atan2( dy, dx );
						double dxn = path[k-1].x - path[sk_last].x;
						double dyn = path[k-1].y - path[sk_last].y;
						double sangn = std::atan2( dyn, dxn );
						ROS_DEBUG("[Breadcrumb] sparse [a,an]: [%0.2f;%0.2f]", sang, sangn);

						//If the angles aren't the same (give or take a bit)
						if( fabs(sangn - sang) > 0.001 ) {
							//Then point k is the end of the line
							res.path_sparse.poses.push_back( res.path.poses.back() );
							sk_last = k;
							ROS_DEBUG("[Breadcrumb] sparse end");
						}
					} else if(k == (path.size()-1) ) {
						//Then this is the initial point, so create the start
						res.path_sparse.poses.push_back( res.path.poses.back() );
					} else if(k == 0) {
						//Then this is the final point, so create the end
						res.path_sparse.poses.push_back( res.path.poses.back() );
					}
				}
			}

			if( param_calc_sparse_ ) {
				ROS_DEBUG("[Breadcrumb] Sparse solution reduced %d points to %d", (int)res.path.poses.size(), (int)res.path_sparse.poses.size() );
			}
		} else {
			ROS_ERROR("[Breadcrumb] No possible solution found!");
		}
	} else if(path.size() == 1) {
		ROS_DEBUG("[Breadcrumb] 1-step path detected, no planning required!");
		geometry_msgs::Pose start;
		geometry_msgs::Pose finish;
		start.position = req.start;
		start.orientation.w = 1;
		finish.position = req.end;
		finish.orientation.w = 1;

		res.path.poses.push_back(start);
		res.path.poses.push_back(finish);
		res.path_sparse.poses.push_back(start);
		res.path_sparse.poses.push_back(finish);
	} else {
		ROS_ERROR("[Breadcrumb] Path finding failed to run!");
	}

	return true;
}

void Breadcrumb::callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg_in) {
	frame_id_ = msg_in->header.frame_id;
	map_info_ = msg_in->info;
	astar_.setWorldSize({(int)msg_in->info.width, (int)msg_in->info.height});
	astar_.clearCollisions();

	// 1) Collect occupied cells first (so we can inflate them)
	std::vector<std::pair<int,int>> occupied_cells;
	occupied_cells.reserve(msg_in->info.width * msg_in->info.height / 10);

	for (int j = 0; j < (int)msg_in->info.height; ++j) {
		for (int i = 0; i < (int)msg_in->info.width; ++i) {
			const int idx = i + j * msg_in->info.width;

			// World center (for bounds mask)
			double wx = (i + 0.5) * msg_in->info.resolution + msg_in->info.origin.position.x;
			double wy = (j + 0.5) * msg_in->info.resolution + msg_in->info.origin.position.y;

			bool out_of_box = false;
			if (enforce_bounds_) {
				out_of_box = (wx < x_min_ || wx > x_max_ || wy < y_min_ || wy > y_max_);
			}

			const int v = msg_in->data[idx];           // -1 unknown, 0..100 known
			const bool unknown  = (v < 0);
			const bool occupied = (v > param_obstacle_threshold_);

			// Treat unknown/occupied/out_of_box as "base obstacles"
			if (out_of_box || unknown || occupied) {
				occupied_cells.emplace_back(i, j);
			}
		}
	}

	// 2) Inflate those cells by a fixed radius in meters
	const double INFLATE_M = 0.2; // desired extra clearance
	const int r = std::max(1, (int)std::ceil(INFLATE_M / msg_in->info.resolution));
	const int W = (int)msg_in->info.width;
	const int H = (int)msg_in->info.height;

	// Optional: mask to avoid duplicate inserts
	std::vector<uint8_t> mask(W * H, 0);

	// Seed mask with the base occupied cells
	for (auto [i,j] : occupied_cells) {
		if (i >= 0 && i < W && j >= 0 && j < H) mask[i + j*W] = 1;
	}

	// Grow each obstacle by 'r' cells (disc-shaped neighborhood)
	for (auto [ci,cj] : occupied_cells) {
		for (int dj = -r; dj <= r; ++dj) {
			for (int di = -r; di <= r; ++di) {
				if (di*di + dj*dj > r*r) continue; // circle
				int ni = ci + di, nj = cj + dj;
				if (ni < 0 || ni >= W || nj < 0 || nj >= H) continue;

				// (Optional) keep the bounds mask strict:
				if (enforce_bounds_) {
					double wx, wy; cellToWorld_(ni, nj, wx, wy);
					if (wx < x_min_ || wx > x_max_ || wy < y_min_ || wy > y_max_) continue;
				}

				mask[ni + nj*W] = 1;
			}
		}
	}

	// 3) Commit the inflated obstacles to A*
	for (int j = 0; j < H; ++j) {
		for (int i = 0; i < W; ++i) {
			if (mask[i + j*W]) {
				astar_.addCollision({i, j});
			}
		}
	}

	if(!flag_got_grid_) {
		flag_got_grid_ = true;
		srv_request_path_ = nhp_.advertiseService("request_path", &Breadcrumb::request_path, this);
		ROS_INFO("[Breadcrumb] Received a new occupancy grid, path planning service started!");
	}
}

bool Breadcrumb::snapToNearestFree_(int& i, int& j) const {
	// Already free?
	if (!astar_.detectCollision({i, j})) return true;

	const int max_r = 50; // ~5m if resolution = 0.1
	for (int r = 1; r <= max_r; ++r) {
		for (int dx = -r; dx <= r; ++dx) {
			for (int dy = -r; dy <= r; ++dy) {
				// ring (prefer border of the square)
				if (std::abs(dx) != r && std::abs(dy) != r) continue;

				const int ni = i + dx;
				const int nj = j + dy;

				if (ni < 0 || nj < 0 ||
					ni >= (int)map_info_.width || nj >= (int)map_info_.height) continue;

				// check world bounds if enforced (though the grid has been masked already)
				if (enforce_bounds_) {
					double wx, wy; cellToWorld_((int)ni, (int)nj, wx, wy);
					if (wx < x_min_ || wx > x_max_ || wy < y_min_ || wy > y_max_) continue;
				}

				if (!astar_.detectCollision({ni, nj})) {
					i = ni; j = nj;
					return true;
				}
			}
		}
	}
	return false;
}