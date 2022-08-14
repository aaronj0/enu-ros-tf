//
// Created by aaronjomyjoseph on 5/11/21.
//

// TODO : Add support to get ENU frame values for emergentLastKnownPos
#include <iostream>
#include <vector>
#include "enu_transforms/enu_transforms.h"
#include <grid_map_ros/grid_map_ros.hpp>

namespace enu_transforms {

    void ENUtransforms::runENU() {
        std::setprecision(9);
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
        ROS_INFO("~~~ENU TRANSFORMS CLASS ACTIVATED~~~\n");
//        std::cout << "Home height set to : " << home_height << "\n";
        double n_origin, e_origin, u_origin;
        double initial_latitude, initial_longitude, initial_altitude;
        gps_ref_origin << min_lat, min_long, home_height;
        gps_origin << min_lat, min_long;
        ENU_geodetic_obj_.initialiseReference(gps_ref_origin);

        double boundary_altitude = 0;

        ROS_INFO("Setting geodetic conversion reference with : Min Lat: %f Min Long: %f Height: %f", min_lat, min_long,
                 home_height);
//        ENU_geodetic_obj_.geodetic2Enu(min_lat, min_long, home_height, &n_origin, &e_origin, &u_origin);

//        ROS_INFO("Geodetic conversion reference set to : East: %f North: %f Up: %f", n_origin, e_origin, u_origin);
//        ENU_geodetic_obj_.enu2Geodetic(n_origin, e_origin, u_origin, &initial_latitude, &initial_longitude,
//                                       &initial_altitude);
//        ROS_INFO("Reconversion to geodetic from ENU reference : Lat: %f Longitude: %f Height: %f", initial_latitude,
//                 initial_longitude, initial_altitude);

        double n, e, u;

        // Boundary points being converted to ENU
        for (int i = 0; i < boundary_points_2d.size(); i += 1) {
            double boundary_point_lat = boundary_points_2d[i][0];
            double boundary_point_long = boundary_points_2d[i][1];
            ENU_geodetic_obj_.geodetic2Enu(boundary_point_lat, boundary_point_long, boundary_altitude, &e, &n, &u);
            boundary_points_2d_enu.push_back({e, n, u});
        }

//        std::cout << "Radius inflation by " << inflate_rd << " feet\n";
//        std::cout << "Height inflation by " << inflate_ht << " feet\n";

        //Inflation and Polygon generation for map_inflate
        for (int i = 0; i < stationary_obstacles_2d.size(); i += 1) {
            double obs_lat = stationary_obstacles_2d[i][0];
            double obs_long = stationary_obstacles_2d[i][1];
            double obs_rad = stationary_obstacles_2d[i][2];
            double obs_height = stationary_obstacles_2d[i][3];

            obs_rad = obs_rad + inflate_rd;
            obs_height = obs_height + inflate_ht;

            Eigen::Vector2d obs_cent(obs_lat, obs_long);
            std::vector<std::vector<double>> obs_poly = GPSObstPoly(obs_cent, obs_rad);
            obstacle_container obs = {obs_height, obs_poly};
            obstacle_container obs_enu = geoObstoENU(obs);
            obstacles_vect.push_back(obs_enu);
        }

        //Polygon generation for map
        for (int i = 0; i < stationary_obstacles_2d.size(); i += 1) {
            double obs_lat = stationary_obstacles_2d[i][0];
            double obs_long = stationary_obstacles_2d[i][1];
            double obs_rad = stationary_obstacles_2d[i][2];
            double obs_height = stationary_obstacles_2d[i][3];
            Eigen::Vector2d obs_cent(obs_lat, obs_long);
            std::vector<std::vector<double>> obs_poly = GPSObstPoly(obs_cent, obs_rad);
            obstacle_container obs_gps = {obs_height, obs_poly};
            obstacles_gps.push_back(obs_gps);
        }

        // GPS Obstacle polygons for QGC visualisation to be passed to link_uav( mavsdk::Geofence )
        for (int i = 0; i < stationary_obstacles_2d.size(); i += 1) {
            double obs_lat = stationary_obstacles_2d[i][0];
            double obs_long = stationary_obstacles_2d[i][1];
            double obs_rad = stationary_obstacles_2d[i][2];
            double obs_height = stationary_obstacles_2d[i][3];
            Eigen::Vector2d obs_cent(obs_lat, obs_long);
            std::vector<std::vector<double>> obs_poly = GPSObstPoly(obs_cent, obs_rad);
            obstacle_container obs = {obs_height, obs_poly};
            obstacle_container obs_enu = geoObstoENU(obs);
            obstacles_vect_uninflated.push_back(obs_enu);
        }

        /// Waypoints conversion to ENU
        for (int i = 0; i < waypoints_2d.size(); i += 1) {
            double obs_lat = waypoints_2d[i][0];
            double obs_long = waypoints_2d[i][1];
            double obs_height = waypoints_2d[i][2];
            ENU_geodetic_obj_.geodetic2Enu(obs_lat, obs_long, obs_height, &e, &n, &u);
            waypoints_2d_enu.push_back({e, n, u});
        }
        /// SearchGrid points
        for (int i = 0; i < search_grid_points_2d.size(); i += 1) {
            double search_grid_point_lat = search_grid_points_2d[i][0];
            double search_grid_point_long = search_grid_points_2d[i][1];
            ENU_geodetic_obj_.geodetic2Enu(search_grid_point_lat, search_grid_point_long, boundary_altitude, &e, &n,
                                           &u);
            search_grid_points_2d_enu.push_back({e, n, u});
        }

        /// Map Centre Position
//        std::cout << "\nMap centre position being converted to ENU:\n";
//        std::cout << "Point: LAT      LONG     HEIGHT\n";
        double pos_lat = map_center_pos_vector[0];
        double pos_long = map_center_pos_vector[1];
        // TODO : The Z value for map_center is actually the value of height of the projection and is not an altitude
        double pos_height = map_center_pos_vector[2];

        std::cout << "Point: " << pos_lat << " " << pos_long << " " << pos_height << "\n";
        ENU_geodetic_obj_.geodetic2Enu(pos_lat, pos_long, pos_height, &e, &n, &u);
        map_center_pos_enu.push_back(e);
        map_center_pos_enu.push_back(n);
        map_center_pos_enu.push_back(u);

//        std::cout << "\nMap centre position in ENU:\n";
//        std::cout << "ENU: " << map_center_pos_enu[0] << " " << map_center_pos_enu[1] << " " << map_center_pos_enu[2]
//                  << " ";
//        std::cout << "\n";

        /// Lost Comms Position
//        std::cout << "\nLost Comms position being converted to ENU:\n";
//        std::cout << "Point: LAT      LONG     HEIGHT\n";
        pos_lat = lost_comms_pos_vector[0];
        pos_long = lost_comms_pos_vector[1];
        pos_height = lost_comms_pos_vector[2];

        std::cout << "Point: " << pos_lat << " " << pos_long << " " << pos_height << "\n";
        ENU_geodetic_obj_.geodetic2Enu(pos_lat, pos_long, pos_height, &e, &n, &u);
        lost_comms_pos_enu.push_back(e);
        lost_comms_pos_enu.push_back(n);
        lost_comms_pos_enu.push_back(u);

//        std::cout << "\nLost Comms position in ENU:\n";
//        std::cout << "ENU: " << lost_comms_pos_enu[0] << " " << lost_comms_pos_enu[1] << " " << lost_comms_pos_enu[2]
//                  << " ";
//        std::cout << "\n";

        /// Air Drop position
//        std::cout << "\nAir Drop position being converted to ENU:\n";
//        std::cout << "Point: LAT      LONG     HEIGHT\n";
        pos_lat = air_drop_pos_vector[0];
        pos_long = air_drop_pos_vector[1];
        pos_height = air_drop_pos_vector[2];

//        std::cout << "Point: " << pos_lat << " " << pos_long << " " << pos_height << "\n";
        ENU_geodetic_obj_.geodetic2Enu(pos_lat, pos_long, pos_height, &e, &n, &u);
        air_drop_pos_enu.push_back(e);
        air_drop_pos_enu.push_back(n);
        air_drop_pos_enu.push_back(u);

//        std::cout << "\nAir Drop position in ENU:\n";
//        std::cout << "ENU: " << air_drop_pos_enu[0] << " " << air_drop_pos_enu[1] << " " << air_drop_pos_enu[2] << " ";
//        std::cout << "\n";
//        std::cout << "\n";

        /// BLOCK TO EXTRACT MIN MAX AND RANGE TERMS FROM 2D
        double min_north = -1, min_east = -1, max_north = -1, max_east = -1;
        for (auto &boundary_point: boundary_points_2d_enu) {
            if (min_north == -1) {
                max_east = min_east = boundary_point[0];
                max_north = min_north = boundary_point[1];
            } else {
                max_east = max(max_east, boundary_point[0]);
                min_east = min(min_east, boundary_point[0]);
                max_north = max(max_north, boundary_point[1]);
                min_north = min(min_north, boundary_point[1]);
            }
        }

//        ROS_INFO("MIN MAX LAT LONG AND ENU PARAMS IN ENU TRANSFORMS:");
//        ROS_INFO("MAX LAT: %f MIN LAT: %f MAX LONG: %f MIN LONG: %f \n", max_lat, min_lat, max_long, min_long);
//        ROS_INFO("MAX NORTH: %f MIN NORTH: %f MAX EAST: %f MIN EAST: %f \n", max_north, min_north, max_east, min_east);
//        ROS_INFO("OFFSET: %d RESOLUTION: %f MULTIPLIER: %d \n", offset_, resolution_, multiplier_);

        min_north_ = min_north - offset_ * resolution_ / multiplier_;
        range_north_ = (max_north - min_north) * multiplier_ + offset_ * 2 * resolution_;
        min_east_ = min_east - offset_ * resolution_ / multiplier_;
        range_east_ = (max_east - min_east) * multiplier_ + offset_ * 2 * resolution_;
    }

    void ENUtransforms::setParams(int offset_set, double resolution_set, int multiplier_set) {
        offset_ = offset_set;
        resolution_ = resolution_set;
        multiplier_ = multiplier_set;
    }

    double ENUtransforms::max(const double &a, const double &b) {
        return a > b ? a : b;
    }

    double ENUtransforms::min(const double &a, const double &b) {
        return a < b ? a : b;
    }

    grid_map::Position ENUtransforms::GPStoGridMap(const double &latitude, const double &longitude) {
        double e, n, u;
        double altitude = 0;
        ENU_geodetic_obj_.geodetic2Enu(latitude, longitude, altitude, &e, &n, &u);
        grid_map::Position gmap = ENUtoMap(e, n);
        return gmap;
    }

    grid_map::Position ENUtransforms::ENUtoMap(const double &east, const double &north) {
        return grid_map::Position{(east - min_east_) * multiplier_ - range_east_ / 2,
                                  (north - min_north_) * multiplier_ - range_north_ / 2};
    }

    coordsW ENUtransforms::posToENU(const double &x, const double &y, const double &z) {
        return {(x + range_east_ / 2) / multiplier_ + min_east_, (y + range_north_ / 2) / multiplier_ + min_north_,
                (z)};
    }

    coordsW ENUtransforms::ENUtoGPS(double e, double n, double u) {
        double lat, longitude, altitude;
        ENU_geodetic_obj_.enu2Geodetic(e, n, u, &lat, &longitude, &altitude);
        return {lat, longitude, altitude};
    }


    Eigen::Vector3d ENUtransforms::enuReferenceVector(const Eigen::Vector3d &enu) {
        Eigen::Vector3d delta = enu_ref_origin - enu;
        Eigen::Vector3d ref_vector = delta.normalized();
        return ref_vector;
    }


    Eigen::Vector2d ENUtransforms::gpsReferenceVector(const Eigen::Vector2d &gps) {
        Eigen::Vector2d delta = gps_origin - gps;
        Eigen::Vector2d ref_vector = delta.normalized();
        return ref_vector;
    }

    double ENUtransforms::haversine(Eigen::Vector2d point1, Eigen::Vector2d point2) {
        double haversine;
        double temp;
        double dist_km;
        double lat1 = point1.x() * DEG_TO_RAD;
        double lon1 = point1.y() * DEG_TO_RAD;
        double lat2 = point2.x() * DEG_TO_RAD;
        double lon2 = point2.y() * DEG_TO_RAD;
        haversine = (pow(sin((1.0 / 2) * (lat2 - lat1)), 2)) +
                    ((cos(lat1)) * (cos(lat2)) * (pow(sin((1.0 / 2) * (lon2 - lon1)), 2)));
        temp = 2 * asin(min(1.0, sqrt(haversine)));
        dist_km = EARTH_RADIUS * temp;
        double dist_ft = dist_km * 3280.84;
        return dist_ft;
    }

    double ENUtransforms::getBearing(Eigen::Vector2d point1, Eigen::Vector2d point2) {
        double lat1 = point2.x() * DEG_TO_RAD;
        double lon1 = point2.y() * DEG_TO_RAD;
        double lat2 = point1.x() * DEG_TO_RAD;
        double lon2 = point1.y() * DEG_TO_RAD;

        double dLon = (lon2 - lon1);
        double y = sin(dLon) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1)
                                           * cos(lat2) * cos(dLon);
        double brng = atan2(y, x);

        brng = brng * RAD_TO_DEG;
        brng = ((int) brng + 360) % 360;
        return brng;

        return brng;
    }

    Eigen::Vector2d ENUtransforms::inverseHaversine(Eigen::Vector2d gps, double bearing, double dist_ft) {
        double lat1 = gps.x() * DEG_TO_RAD;
        double lon1 = gps.y() * DEG_TO_RAD;

        double dist_km = dist_ft / 3280.84;
        double dist_angular = dist_km / EARTH_RADIUS;
        double theta = bearing * DEG_TO_RAD;
        double lat2 = asin(sin(lat1) * cos(dist_angular) + cos(lat1) * sin(dist_angular) * cos(theta));
        double lon2 =
                lon1 + atan2(sin(theta) * sin(dist_angular) * cos(lat2), cos(dist_angular) - sin(lat2) * sin(lat2));
        lat2 = lat2 * RAD_TO_DEG;
        lon2 = lon2 * RAD_TO_DEG;
        Eigen::Vector2d gps_obst_point(lat2, lon2);
        return gps_obst_point;
    }

    std::vector<std::vector<double>>
    ENUtransforms::genPoly(Eigen::Vector2d rad_point, Eigen::Vector2d centre_point, int n) {
        std::vector<std::vector<double>> poly;
        double center_lat = centre_point.x();
        double center_long = centre_point.y();
        double rad_lat = rad_point.x();
        double rad_long = rad_point.y();

        double radius = haversine(rad_point, centre_point);
        double rad_metres = radius / 3.28084;

        for (int k = 0; k < n; k++) {
            double theta = M_PI * 2 * k / n;
            double dx = rad_metres * cos(theta);
            double dy = rad_metres * sin(theta);
            std::vector<double> point;
            point.push_back(center_lat + (180 / M_PI) * (dy / (EARTH_RADIUS * 1000)));
            point.push_back(center_long + (180 / M_PI) * (dx / (EARTH_RADIUS * 1000)) / cos(center_lat * M_PI / 180));
            poly.push_back(point);
        }
        poly.push_back(poly[0]);
        return poly;
    }

    std::vector<std::vector<double>> ENUtransforms::GPSObstPoly(Eigen::Vector2d obst_centre, double radius) {
        double bearing = getBearing(gps_origin, obst_centre);
        Eigen::Vector2d rad_point = inverseHaversine(obst_centre, bearing, radius);
        Eigen::Vector2d delta_rad(obst_centre.x() - rad_point.x(), obst_centre.y() - rad_point.y());
        Eigen::Vector2d rad_point_further = rad_point + 2 * delta_rad;
        std::vector<std::vector<double>> polygon = genPoly(rad_point, obst_centre, SIDES_OF_POLYGON);
        return polygon;
    }

    obstacle_container ENUtransforms::geoObstoENU(obstacle_container ob) {
        double obs_height = ob.height;
        obstacle_container obs_enu;
        double e, n, u;
        for (int i = 0; i < ob.poly.size(); i += 1) {
            double obs_lat = ob.poly[i][0];
            double obs_long = ob.poly[i][1];
            ENU_geodetic_obj_.geodetic2Enu(obs_lat, obs_long, obs_height, &e, &n, &u);
            obs_enu.poly.push_back({e, n});
        }
        obs_enu.height = obs_height;
        return obs_enu;
    }
}  // namespace





