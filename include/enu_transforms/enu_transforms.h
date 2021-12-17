//
// Created by aaronjomyjoseph on 5/11/21.
//

#ifndef ENU_TRANSFORMS_INCLUDE_ENU_TRANSFORMS_ENU_TRANSFORMS_H_
#define ENU_TRANSFORMS_INCLUDE_ENU_TRANSFORMS_ENU_TRANSFORMS_H_
#include <fstream>


// #include "msg/mission_file.msg"
// #include "enu_transforms/msg/mission_file.h
// mission for interop parsing

//INTEROP STUFF//interop stuff
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <enu_transforms/MissionFileAction.h>
////

//
#include<cmath>
#include<iostream>
#include<chrono>
#include<algorithm>
#include <nlohmann/json.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseArray.h>

#include </home/maximus/auvsi_ws/src/planner_server/include/lazy_theta_star_3d/coord_types.h>
#include <transforms/geodetic_conv.h>

#include<cstdlib>



namespace enu_transforms {
class ENUtransforms {
public:

    ENUtransforms()
    {

    }
   ~ENUtransforms() = default;
    ENUtransforms(int i);

    // vectors after conversion to 2d
    std::vector<std::vector<double>> boundary_points_2d;
    std::vector<std::vector<double>> waypoints_2d;
    std::vector<std::vector<double>> stationary_obstacles_2d;
    // enu vectors
    std::vector<std::vector<double>> boundary_points_2d_enu;
    std::vector<std::vector<double>> waypoints_2d_enu;
    std::vector<std::vector<double>> stationary_obstacles_2d_enu;




  geodetic_converter::GeodeticConverter ENU_geodetic_obj_;
  grid_map::Position GPStoGridMap(const double & latitude, const double & longitude);
  grid_map::Position ENUtoMap(const double &north, const double &east);
  coordsW posToENU(const double & x, const double & y, const double & z);
  coordsW ENUtoGPS(double e, double n, double u);
  void Run();
  void Set_Params(int offset_, double resolution_, int multiplier);
  // ENUtransforms::feet_to_ENU(const double &north, const double &east) const;
  // enu_transforms::mission_file msg;
  double max(const double & a, const double & b);
  double min(const double & a, const double & b);
  // void publish();

  double min_lat_;
  double min_long_;
  double range_lat_;
  double range_long_;
  double min_north_;
  double min_east_;
  double range_north_;
  double range_east_;
  int multiplier_;
  int offset;
  double resolution;
private:
    ros::NodeHandle *nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber path_sub_;
    ros::Publisher map_pub_;

};
}

#endif 
