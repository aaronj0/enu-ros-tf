
//
// Created by aaronjomyjoseph
//

#ifndef ENU_TRANSFORMS_INCLUDE_ENU_TRANSFORMS_ENU_TRANSFORMS_H_
#define ENU_TRANSFORMS_INCLUDE_ENU_TRANSFORMS_ENU_TRANSFORMS_H_

#include <fstream>
#include<cmath>
#include<iostream>
#include<chrono>
#include<algorithm>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <transforms/coord_types.h>
#include <transforms/geodetic_conv.h>
#include<cstdlib>

namespace enu_transforms {
    class ENUtransforms {
    public:

        ENUtransforms() {
        }

        ~ENUtransforms() = default;

        const int SIDES_OF_POLYGON = 30; //Sides of polygon generated for stationary obstacle_container
        const double pi = M_PI;
        const double EARTH_RADIUS = 6372.79756085;
        const double DEG_TO_RAD = pi / 180;
        const double RAD_TO_DEG = 180 / pi;

        double min_north_;
        double min_east_;
        double range_north_;
        double range_east_;
        int inflate_rd;
        int inflate_ht;
        int multiplier_;
        int offset_;
        double resolution_;

        Eigen::Vector3d enu_ref_origin;
        Eigen::Vector3d gps_ref_origin;
        Eigen::Vector2d gps_origin;

        // vectors after conversion to 2d
        std::vector<std::vector<double>> boundary_points_2d;
        std::vector<std::vector<double>> waypoints_2d;
        std::vector<std::vector<double>> stationary_obstacles_2d;
        std::vector<std::vector<double>> search_grid_points_2d;
        std::vector<double> map_center_pos_vector;
        std::vector<double> lost_comms_pos_vector;
        std::vector<double> air_drop_pos_vector;

        //enu vectors
        std::vector<std::vector<double>> boundary_points_2d_enu;
        std::vector<std::vector<double>> waypoints_2d_enu;
        std::vector<obstacle_container> obstacles_vect;
        std::vector<obstacle_container> obstacles_vect_uninflated;
        std::vector<obstacle_container> obstacles_gps;
        std::vector<std::vector<double>> search_grid_points_2d_enu;
        std::vector<double> map_center_pos_enu;
        std::vector<double> lost_comms_pos_enu;
        std::vector<double> air_drop_pos_enu;

        geodetic_converter::GeodeticConverter ENU_geodetic_obj_;


        /**
        * @brief Function to set the offset_, multiplier_ and precision parameters required for all linear transformation equations
        * using min and mix bounds as seen in potential_grid
        * Call this function in any package after initialising its object before running ENUTransforms::Run()
        * @param offset_set Set params from either planner_server or potential_grid
        * @param resolution_set
        * @param multiplier_set
        */
        void setParams(int offset_set, double resolution_set, int multiplier_set);

        /**
         * @brief Main driver code for the package. Call to run the transforms.
         */
        void runENU();

        static double max(const double &a, const double &b);

        static double min(const double &a, const double &b);

        /**
         * @brief Function to convert the GPS co ordinates to GridMap co ordinates via the ENU transformation object
         * @param latitude
         * @param longitude
         * @return A grid_map::Position type {x,y}
         */
        grid_map::Position GPStoGridMap(const double &latitude, const double &longitude);

        /**
         * @brief Function that uses the same equations from potential_grid to convert
         * the ENU values into a GridMap frame using min and max bounds
         * @param north
         * @param east
         * @return
         */
        grid_map::Position ENUtoMap(const double &north, const double &east);

        /**
        * @brief Conversion function from GridMap position back to ENU.
        * Used in Planner server to reconvert GridMap plan to ENU
        * @param x Gridmap pos x
        * @param y Gridmap pos y
        * @param z Gridmap pos z
        * @return coordsW type of {e,n,u}
        */
        coordsW posToENU(const double &x, const double &y, const double &z);

        /**
        * @brief Conversion function for ENU to GPS. Uses Geodetic conversion object
        * @param e Input ENU north value
        * @param n Input ENU east value
        * @param u Input ENU height
        * @return coordsW type of {lat,long,alt}
        */
        coordsW ENUtoGPS(double e, double n, double u);

        /**
         * @brief Returns a reference unit vector from the set reference origin to a give enu point
         * @param enu Eigen Vector3d consisting of east, north, up
         * @return Returns a reference unit vector from the set reference origin to a give enu point
         */
        Eigen::Vector3d enuReferenceVector(const Eigen::Vector3d &enu);

        /**
         * @brief Function to calculate the distance between two lat long using the Haversine function
         * @param point1
         * @param point2
         * @return Distance in feet
         */
        double haversine(Eigen::Vector2d point1, Eigen::Vector2d point2);

        /**
        * @brief Generate Unit vector
        * @param gps Eigen Vector2d consisting of lat, long
        * @return Returns a reference unit vector from the set reference origin to a given gps point
        */
        Eigen::Vector2d gpsReferenceVector(const Eigen::Vector2d &gps);

        /**
         * @brief A function that calculates the bearing between two GPS coordinates
         * @param point1 Eigen Vector2d gps point 1
         * @param point2 Eigen Vector2d gps point 2
         * @return bearing angle in degrees(clockwise from North)
         */
        double getBearing(Eigen::Vector2d point1, Eigen::Vector2d point2);

        /**
         * @brief Inverse Haversine function that returns the lat long of the point that is distance d away,
         * and at a bearing angle theta away.
         * @param gps GPS point from which it is calculated
         * @param bearing bearing angle in degrees(clockwise north)
         * @param dist_ft Distance
         * @return  Eigen Vector2d containing the GPS point at that distance away
         */
        Eigen::Vector2d inverseHaversine(Eigen::Vector2d gps, double bearing, double dist_ft);

        /**
         * @brief Function that generates a polygon as a 2d vector given the radial point, centre point and number of vertices
         * @param rad_point Radial Point(found using inverseHaversine()
         * @param centre_point Centre of obstacle_container
         * @param n Number of vertices of generated polygon
         * @return
         */
        std::vector<std::vector<double>> genPoly(Eigen::Vector2d rad_point, Eigen::Vector2d centre_point, int n);

        /**
         * @brief Direct function to generate GPS polygon for stationary obstacles.
         * @param obst_centre JSON obstacle_container centre
         * @param radius radius in feet
         * @return 2d vector polygon
         */
        std::vector<std::vector<double>> GPSObstPoly(Eigen::Vector2d obst_centre, double radius);

        /**
         *
         * @param ob
         * @return
         */
        obstacle_container geoObstoENU(obstacle_container ob);
    };
}

#endif
