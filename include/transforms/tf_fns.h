// #ifndef ENU_TRANSFORMS_TRANSFORMS_TF_FNS_H
// #define ENU_TRANSFORMS_TRANSFORMS_TF_FNS_H

// #include<cmath>
// #include<iostream>
// #include <nlohmann/json.hpp>
// #include <fstream>
// #include <ros/package.h>
// #include<chrono>
// #include<algorithm>
// #include<ros/ros.h>
// #include<nav_msgs/OccupancyGrid.h>
// #include<geometry_msgs/Pose.h>
// #include "grid_map_ros/GridMapRosConverter.hpp"
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <lazy_theta_star_3d/coord_types.h>
// #include<geometry_msgs/PoseArray.h>
// #include<cstdlib>

// // TODO (aaronjomyjoseph) : Include ENU object calls thats created in enu_transforms.cpp

// namespace util_tf_fns {
// class tfFns {
//   int offset;
//   int multiplier_;
//   double precision_;
//   double min_lat_;
//   double min_long_;
//   double max_lat_;
//   double max_long_;
//   double range_lat_;
//   double range_long_;
//   double min_north_;
//   double max_north_;
//   double min_east_;
//   double max_east_;


// public:

//   double max(const double & a, const double & b){
// 	return a > b ? a : b;
//   }

//   double min(const double & a, const double & b){
// 	return a < b ? a : b;
//   }

// // CHANGES (aaronjomyjoseph): This data should come from potential grid instead of planner server.
// // Replace all calls in planner_server to planner_server/interfacing_stuff/tf_fns.h functions to calling tf_fns fromn the transforms package
// // this will streamline all transforms function calls in planner_server
   
//    tfFns(std::vector<float> mission ):
// {
	
// 	auto boundary_points = mission["flyZones"][0]["boundaryPoints"];

// 	double resolution = std::pow(10, precision_);

// 	// min_lat_ = min_lat - offset * resolution / multiplier_;
// 	// range_lat_ = (max_lat - min_lat) * multiplier_ + offset * 2 * resolution;
// 	// min_long_ = min_long - offset * resolution / multiplier_;
// 	// range_long_ = (max_long - min_long) * multiplier_ + offset * 2 * resolution;

// 	ROS_INFO("Transformation Function params: min_lat - %f \n max_lat - %f \n min_long - %f \n max_long - %f and the offsets % f | %f", min_lat, max_lat, min_long, max_long, min_lat_, min_long_);

//   }

// 	grid_map::Position PotentialGrid::ENUtoMap(const double &north, const double &east) const 
//   {
//   return grid_map::Position{(north - min_north_) * multiplier_ - range_north_ / 2, (east - min_east_ ) * multiplier_ - range_east_ / 2};
// 	}


//   coordsW posToENU(const double & x, const double & y, const double & z)
// {
// 	return {(x + range_north_ / 2) / multiplier_ + min_north_, (y + range_east_ / 2) / multiplier_ + min_east_,
// 			z };
  
// }
//  coordsW ENUtoGPS(const double & x, const double & y, const double & z)
//  {
// 	 ENU_geodetic_obj_.enu2Geodetic(0, 0, 0, &x, &y, &z);
	 
//  }
// };

// }  //namesp
// namespace util_tf_fns

// #endif //ENU_TRANSFORMS_TRANSFORMS_TF_FNS_H