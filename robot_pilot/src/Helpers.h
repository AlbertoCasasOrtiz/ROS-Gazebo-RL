//
// Created by alberto on 6/09/19.
//

#ifndef SRC_HELPERS_H
#define SRC_HELPERS_H

#include "nav_msgs/Odometry.h"

class Helpers {
public:
    /// Get yaw angle of the robot in degrees.
    /// \param msg Message from odometry.
    /// \return Yaw angle in degrees.
    static int getYaw(const nav_msgs::Odometry::ConstPtr& msg);

};


#endif //SRC_HELPERS_H
