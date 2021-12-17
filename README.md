# ENU-ROS-Transformations package
An ENU tangential plane ROS Transformations package that allows for navigation stacks and planners to use that accounts for curvature and ellipsoid behavior-based inaccuracies due to standard geodetic data.

This package is a geodetic conversion package created for the AUVSI SUAS navigation stack.
It runs various conversions between GPS, ECEF, NED and ENU co ordinate frames.
This linearises the co-ordinate frame of any arbitrary map or data structure used by a planner while maintaining the appropriate references to geodetic curvature with a return to the first-order model of the earth as being flat, where they serve as  local  reference  directions  for  representing  vehicle  attitude  and  velocity  for operation  on  or  near  the  surface  of  the  earth
For AUVSI purposes the frame used is ENU, a local tangent plane(LTP) system.
To change the different geodetic systems, functions in include/transforms/geodetic_conv.h can be used.
