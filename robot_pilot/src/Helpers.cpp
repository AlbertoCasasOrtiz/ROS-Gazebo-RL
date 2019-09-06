//
// Created by alberto on 6/09/19.
//

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "Helpers.h"

int Helpers::getYaw(const nav_msgs::Odometry::ConstPtr &msg) {
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return (int)((yaw + M_PI)*180/M_PI);
}
