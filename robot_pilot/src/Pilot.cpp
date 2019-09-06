//
// Created by alberto on 6/09/19.
//

#include "Pilot.h"
#include "Helpers.h"

Pilot::Pilot(int argc, char **argv) {
    // Initializations
    heading = Pilot::Dir::UP;
    robotSpeed = robotAngularSpeed = 0.3;
    posX = posY = turnZ = 0;
    flag_once = true;
    flag_notified = true;
    stepDistance = 0.85;

    // ROS starts here.
    ros::init(argc, argv, "pilot");

    // Nodehandler for handle other nodes.
    ros::NodeHandle nh;

    // Inidialize subscribers and publishers
    Pilot::cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    Pilot::commandCompletedPublisher = nh.advertise<std_msgs::String>("/commander", 10);
    Pilot::odomSubscriber = nh.subscribe("/odom", 1, &Pilot::odomCallback, this);
    Pilot::commandSubscriber = nh.subscribe("/commander", 1, &Pilot::commanderCallback, this);

    // Spin for subscribers.
    ros::spin();
}

void Pilot::commanderCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("Command received: [%s]", msg->data.c_str());
    Pilot::commands.push(Pilot::parseCommand(msg->data));
}

void Pilot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // If there are no commands, stop.
    if(Pilot::commands.empty()) {
        Pilot::stop();
        if(flag_notified) {
            flag_notified = false;

            //Send message
            std_msgs::String str;
            std::stringstream ss;
            ss << "1";
            str.data = ss.str();
            Pilot::commandCompletedPublisher.publish(str);
        }
    } else {
        // Activate flag notified for when the command is complete.
        flag_notified = true;
        // Take first element of the queue.
        switch(commands.front()){
            case Commands::FORWARD:
                Pilot::goForward(msg, Pilot::robotSpeed);
                break;
            case Commands::LEFT:
                Pilot::turnLeft(msg, Pilot::robotAngularSpeed);
                break;
            case Commands::RIGHT:
                Pilot::turnRight(msg, Pilot::robotAngularSpeed);
                break;
            case Commands::STOP:
                Pilot::stop();
                break;
            default:
                Pilot::stop();
                break;
        }
    }
}

void Pilot::goForward(const nav_msgs::Odometry::ConstPtr &msg, float speed) {
    if(Pilot::flag_once) {
        // Initialization of command forward.
        ROS_INFO("FORWARD");
        Pilot::posX = msg->pose.pose.position.x;
        Pilot::posY = msg->pose.pose.position.y;
        Pilot::flag_once = false;

        // Execution of command forward.
        geometry_msgs::Twist cmd;
        cmd.linear.x = speed;
        cmd.angular.z = 0.0;
        Pilot::cmdVelPublisher.publish(cmd);
    }

    // Terminate command forward.
    // If advanced one unit...
    if (fabs(posX - msg->pose.pose.position.x) > Pilot::stepDistance || fabs(posY - msg->pose.pose.position.y) > Pilot::stepDistance) {
        Pilot::flag_once = true;
        Pilot::commands.pop();
    }
}

void Pilot::turnLeft(const nav_msgs::Odometry::ConstPtr &msg, float angularSpeed) {
    //Calculate yaw
    int yaw = Helpers::getYaw(msg);

    if(Pilot::flag_once) {
        //Initialization of command turn left.
        ROS_INFO("LEFT");
        Pilot::updateHeading(Pilot::Commands::LEFT);
        Pilot::flag_once = false;


        // Execution of command turn left.
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = angularSpeed;
        Pilot::cmdVelPublisher.publish(cmd);
    }

    // Terminate command turn left.
    // If end turning...
    if ((int) yaw == (int) Pilot::turnZ) {
        Pilot::flag_once = true;
        Pilot::commands.pop();
    }
}

void Pilot::turnRight(const nav_msgs::Odometry::ConstPtr &msg, float angularSpeed) {
    //Calculate yaw
    int yaw = Helpers::getYaw(msg);

    if (Pilot::flag_once == 1) {
        // Initialization of command turn right.
        ROS_INFO("RIGHT");
        Pilot::updateHeading(Pilot::Commands::RIGHT);
        Pilot::flag_once = false;

        // Execution of command turn right.
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = -angularSpeed;
        Pilot::cmdVelPublisher.publish(cmd);
    }

    // Terminate command turn right.
    // If end turning...
    if ((int) yaw == (int) Pilot::turnZ) {
        Pilot::flag_once = true;
        Pilot::commands.pop();
    }
}

void Pilot::stop() {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    Pilot::cmdVelPublisher.publish(cmd);
}

void Pilot::updateHeading(Pilot::Commands command) {
    if(command == Pilot::Commands::LEFT){
        switch(Pilot::heading){
            case Pilot::Dir::UP:
                Pilot::turnZ = 270;
                Pilot::heading = Pilot::Dir::LEFT;
                break;
            case Pilot::Dir::LEFT:
                Pilot::turnZ = 0;
                Pilot::heading = Pilot::Dir::DOWN;
                break;
            case Pilot::Dir::DOWN:
                Pilot::turnZ = 90;
                Pilot::heading = Pilot::Dir::RIGHT;
                break;
            case Pilot::Dir::RIGHT:
                Pilot::turnZ = 180;
                Pilot::heading = Pilot::Dir::UP;
                break;
        }
    } else if(command == Pilot::Commands::RIGHT){
        switch(Pilot::heading){
            case Pilot::Dir::UP:
                Pilot::turnZ = 90;
                Pilot::heading = Pilot::Dir::RIGHT;
                break;
            case Pilot::Dir::LEFT:
                Pilot::turnZ = 180;
                Pilot::heading = Pilot::Dir::UP;
                break;
            case Pilot::Dir::DOWN:
                Pilot::turnZ = 270;
                Pilot::heading = Pilot::Dir::LEFT;
                break;
            case Pilot::Dir::RIGHT:
                Pilot::turnZ = 0;
                Pilot::heading = Pilot::Dir::DOWN;
                break;
        }

    }
}

Pilot::Commands Pilot::parseCommand(const std::string& command) {
    if(command == "FORWARD")
        return Pilot::Commands::FORWARD;
    else if(command == "LEFT")
        return Pilot::Commands::LEFT;
    else if(command == "RIGHT")
        return Pilot::Commands::RIGHT;
    else if(command == "STOP")
        return Pilot::Commands ::STOP;
    else {
        ROS_ERROR("Received wrong command: [%s]. Valid commands are [FORWARD, LEFT, RIGHT, STOP]", command.c_str());
        return Pilot::Commands ::STOP;
    }

}
