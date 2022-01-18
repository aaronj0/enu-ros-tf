//
// Created by aaronjomyjoseph on 5/11/21.
//

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "enu_transforms/enu_transforms.h"
#include <grid_map_ros/grid_map_ros.hpp>


namespace enu_transforms{
    // ENUtransforms(std::vector<float> interop_mission_data)

        //  enu_vect_pub_ = nh_->advertise<enu_transforms::mission_file>("enu_mission_vector", 1, true);

    void ENUtransforms::Run() {

//// TO RUN WITH MISSION FILE
//     private_nh_.param<std::string>("mission_file", mission_file_, "<INSERT MISSION DIR>");
//     // private_nh_.getParam("mission_file", mission_file_);
//
//     std::string package_path = ros::package::getPath("potential_grid");
//     std::ifstream mission_file(package_path + mission_file_);
//     std::string mission_string((std::istreambuf_iterator<char>(mission_file)),
//                              std::istreambuf_iterator<char>());
//     nlohmann::json interop_mission_data = nlohmann::json::parse("<INSERT MISSION DIR>");
//
//// TO RUN WITH INTEROP DATA

//            res_enu = res;
//            stationary_obstacles_vector = res_enu->stationaryObstacles;
//            boundary_points_vector = res_enu->flyZones;
//            waypoints_vector = res_enu->waypoints;
////
//            for (int i = 0; i < boundary_points_vector.size(); i += 2) {
//                boundary_points_2d.push_back({boundary_points_vector[i], boundary_points_vector[i + 1]});
//            }
//            for (int i = 0; i < waypoints_vector.size(); i += 3) {
//                waypoints_2d.push_back({waypoints_vector[i], waypoints_vector[i + 1], waypoints_vector[i + 2]});
//            }
//            for (int i = 0; i < stationary_obstacles_vector.size(); i += 4) {
//                stationary_obstacles_2d.push_back({stationary_obstacles_vector[i], stationary_obstacles_vector[i + 1],
//                                                   stationary_obstacles_vector[i + 2], stationary_obstacles_vector[i + 3]});
//            }
//            std::cout << "2D VEC FOR MAPPING :\n";
//            std::cout << "Boundary points :\n";
//            for (int i = 0; i < boundary_points_2d.size(); i++) {
//                for (int j = 0; j < boundary_points_2d[i].size(); j++)
//                    std::cout << boundary_points_2d[i][j] << " ";
//                std::cout << std::endl;
//            }
//            std::cout << "Waypoints :\n";
//            for (int i = 0; i < waypoints_2d.size(); i++) {
//                for (int j = 0; j < waypoints_2d[i].size(); j++)
//                    std::cout << waypoints_2d[i][j] << " ";
//                std::cout << std::endl;
//            }
//            std::cout << "Stationary obstacles :\n";
//            for (int i = 0; i < stationary_obstacles_2d.size(); i++) {
//                for (int j = 0; j < stationary_obstacles_2d[i].size(); j++)
//                    std::cout << stationary_obstacles_2d[i][j] << " ";
//                std::cout << std::endl;
//            }

//        auto boundary_points = interop_mission_data["flyZones"][0]["boundaryPoints"];

        double min_lat = -1, min_long = -1, max_lat = -1, max_long = -1;
        for (auto &boundary_point: boundary_points_2d) {
            if (min_lat == -1) {
                max_lat = min_lat = boundary_point[0];
                max_long = min_long = boundary_point[1];
            } else {
                max_lat = max(max_lat, boundary_point[0]);
                min_lat = min(min_lat, boundary_point[0]);
                max_long = max(max_long, boundary_point[1]);
                min_long = min(min_long, boundary_point[1]);
            }
        }

        double home_height = 0;
        std::cout << "ENU TRANSFORMS CLASS ACTIVATED" << "\n";
        std::cout << "Home Height set to : " << home_height << "\n";
        double altitude = 750;
        double n_origin, e_origin, u_origin;
        double initial_latitude, initial_longitude, initial_altitude;
        ENU_geodetic_obj_.initialiseReference(min_lat, min_long, home_height);


        std::cout << "Default height of points without altitude set to : " << altitude << "\n";
        // std::cout <<"Geodetic conversion reference set to : "<<"Lat: "<<min_lat<<" Longitude:"<<min_long<<" Height: "<<home_height<<"\n";
        ROS_INFO("Setting geodetic conversion reference with : Min Lat: %f Min Long: %f Height: %f", min_lat, min_long,
                 home_height);
        ENU_geodetic_obj_.geodetic2Enu(min_lat, min_long, home_height, &n_origin, &e_origin, &u_origin);
        ROS_INFO("Geodetic conversion reference set to : East: %f North: %f Up: %f", n_origin, e_origin, u_origin);
        ENU_geodetic_obj_.enu2Geodetic(n_origin, e_origin, u_origin, &initial_latitude, &initial_longitude,
                                       &initial_altitude);
        ROS_INFO("Reconversion to geodetic from ENU reference : Lat: %f Longitude: %f Height: %f", initial_latitude,
                 initial_longitude, initial_altitude);

        double n, e, u;

        for (auto &boundary_point: boundary_points_2d) {
            double bound_lat = boundary_point[0];
            double bound_long = boundary_point[1];
            ENU_geodetic_obj_.geodetic2Enu(bound_lat, bound_long, altitude, &e, &n, &u);
            std::cout << "North:" << n << " East:" << e << " up :" << u << "\n";

        }
        std::cout << "Boundary points being converted to ENU:\n";
        std::cout << "Point: LAT      LONG \n";
        for (int i = 0; i < boundary_points_2d.size(); i += 1) {
            double boundary_point_lat = boundary_points_2d[i][0];
            double boundary_point_long = boundary_points_2d[i][1];
            std::cout << "Point: " << boundary_point_lat << " " << boundary_point_long << "\n";
            ENU_geodetic_obj_.geodetic2Enu(boundary_point_lat, boundary_point_long, altitude, &e, &n, &u);
            boundary_points_2d_enu.push_back({e, n, u});
        }
        std::cout << "Boundary points in ENU:\n";
        for (int i = 0; i < boundary_points_2d_enu.size(); i++) {
            for (int j = 0; j < boundary_points_2d_enu[i].size(); j++)
                std::cout << boundary_points_2d_enu[i][j] << " ";
            std::cout << std::endl;
        }

        std::cout << "Stationary Obstacles being converted to ENU:\n";
        std::cout << "Point: LAT      LONG     HEIGHT\n";
        for (int i = 0; i < stationary_obstacles_2d.size(); i += 1) {
            double obs_lat = stationary_obstacles_2d[i][0];
            double obs_long = stationary_obstacles_2d[i][1];
            double r = stationary_obstacles_2d[i][2];
            double obs_height = stationary_obstacles_2d[i][3];
            std::cout << "Point: " << obs_lat << " " << obs_long << " " << obs_height << "\n";
            ENU_geodetic_obj_.geodetic2Enu(obs_lat, obs_long, obs_height, &e, &n, &u);
            stationary_obstacles_2d_enu.push_back({e, n, r, u});
        }
        std::cout << "Stationary Obstacles in ENU:\n";
        for (int i = 0; i < stationary_obstacles_2d_enu.size(); i++) {
            for (int j = 0; j < stationary_obstacles_2d_enu[i].size(); j++)
                std::cout << stationary_obstacles_2d_enu[i][j] << " ";
            std::cout << std::endl;
        }

        //// BLOCK TO EXTRACT MIN MAX AND RANGE TERMS FROM 2D
//        double min_lat = -1, min_long = -1, max_lat = -1, max_long = -1;
        double min_north = -1, min_east = -1, max_north = -1, max_east = -1;
        std::cout << "MAX LAT:" << max_lat << " MIN LAT:" << min_lat << " MAX LONG:" << max_long << " MIN LONG:" << min_long<<"\n";
        for (auto &boundary_point: boundary_points_2d_enu) {
            if (min_north == -1) {
                max_north = min_north = boundary_point[0];
                max_east = min_east = boundary_point[1];
            } else {
                max_north = max(max_north, boundary_point[0]);
                min_north = min(min_north, boundary_point[0]);
                max_east = max(max_east, boundary_point[1]);
                min_east = min(min_east, boundary_point[1]);
            }
        }
        std::cout<<"OFFSET:"<< offset<<" RES:"<<resolution<<" MULT"<<multiplier_<<"\n";
        std::cout << "MAX NORTH:" << max_north << " MIN NORTH:" << min_north << " MAX EAST:" << max_east << " MIN EAST:" << min_east<<"\n";
        min_north_ = min_north - offset * resolution / multiplier_;
        range_north_ = (max_north  - min_north)* multiplier_ + offset * 2 * resolution ;
        min_east_ = min_east - offset * resolution / multiplier_;
        range_east_ = (max_east - min_east) * multiplier_ + offset * 2 * resolution ;
        std::cout<<"_ params in enu_tf"<<min_north_<<" "<<range_north_<<" "<<min_east_<<" "<<range_east_<<"\n";



    }

    void ENUtransforms::Set_Params(int offset_pg, double resolution_pg, int multiplier_pg)
    {
        offset = offset_pg;
        resolution = resolution_pg;
        multiplier_ = multiplier_pg;
    }
    double ENUtransforms::max(const double & a, const double & b)
    {
	    return a > b ? a : b;
    }

    double ENUtransforms::min(const double & a, const double & b)
    {
        return a < b ? a : b;
    }

    // TODO CHANGES (aaronjomyjoseph): This data should come from potential grid instead of planner server.
    // TODO Replace all calls in planner_server to planner_server/interfacing_stuff/tf_fns.h functions to calling tf_fns fromn the transforms package
    // TODO this will streamline all transforms function calls in planner_server
    // TODO add shared pointer to tf obj: included from planner server


    grid_map::Position ENUtransforms::GPStoGridMap(const double & lat, const double & longitude)
    {   
        double n,e,u;
        double altitude = 0;
        ENU_geodetic_obj_.geodetic2Enu(lat, longitude, altitude, &e, &n, &u);
        grid_map::Position gmap = ENUtoMap(n, e);
        return gmap;
    }
	grid_map::Position ENUtransforms::ENUtoMap(const double &north, const double &east)
    {
        return grid_map::Position{(north - min_north_) * multiplier_ - range_north_ / 2, (east - min_east_ ) * multiplier_ - range_east_ / 2};

    }

    coordsW ENUtransforms::posToENU(const double & x, const double & y, const double & z)
    {
	    return {(x + range_north_ / 2) / multiplier_ + min_north_, (y + range_east_ / 2) / multiplier_ + min_east_,(z)};
    }
    coordsW ENUtransforms::ENUtoGPS(double e, double n, double u)
    {
        double lat, longitude, altitude;
        ENU_geodetic_obj_.enu2Geodetic(e, n, u, &lat, &longitude, &altitude);
        return {lat , longitude, altitude};

    }


    // ENUtransforms::feet_to_ENU(const double &north, const double &east) const
    // {

    // }
}

//namespace

