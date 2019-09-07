//
// Created by alberto on 6/09/19.
//

#include "Pilot.h"
#include "Helpers.h"

Pilot::Pilot(int argc, char **argv) {
    // Initializations
    heading = Pilot::Dir::UP;
    robotAngularSpeed = 0.3;
    robotAngularSpeed = 0.6
    posX = posY = turnZ = 0;
    stepDistance = 0.7;
    flag_once = true;

    algorithm_initialized = false;

    // ROS starts here.
    ros::init(argc, argv, "pilot");

    // Nodehandler for handle other nodes.
    ros::NodeHandle nh;

    // Inidialize subscribers and publishers
    Pilot::cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Publish status to commander.
    Pilot::commandCompletedPublisher = nh.advertise<std_msgs::String>("/commander", 10);
    Pilot::odomSubscriber = nh.subscribe("/odom", 1, &Pilot::odomCallback, this);
    //Subscribe of pilot for getting actions
    Pilot::commandSubscriber = nh.subscribe("/pilot", 1, &Pilot::commanderCallback, this);

    // Subscribe to range sensors
    Pilot::rangeSubscriberFront = nh.subscribe("/sensor/ir_front", 1, &Pilot::irFrontCallback, this);
    Pilot::rangeSubscriberLeft = nh.subscribe("/sensor/ir_left", 1, &Pilot::irLeftCallback, this);
    Pilot::rangeSubscriberRight = nh.subscribe("/sensor/ir_right", 1, &Pilot::irRightCallback, this);
    Pilot::rangeSubscriberBack = nh.subscribe("/sensor/ir_back", 1, &Pilot::irBackCallback, this);

    // Spin for subscribers.
    ros::spin();
}

void Pilot::commanderCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("RECEIVED: [%s]", msg->data.c_str());
    if(msg->data == "algorithm_initialized"){
        algorithm_initialized = true;
    } else {

        if (algorithm_initialized) {
            Pilot::parseAction(msg->data);
            if (!possibleAction) {
                sendMessage("not possible");
            } else {
                sendMessage("possible");
            }
        }
    }
}

void Pilot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // Advertise the commander that we are ready.
    if(!algorithm_initialized) {
        Pilot::sendMessage("init");
    }

    // If there are no commands, stop.
    if(Pilot::commands.empty()) {
        if(algorithm_initialized) {
            Pilot::stop();
            Pilot::sendMessage("next movement");
        }
    } else {
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

void Pilot::parseAction(const std::string& action) {
    Pilot::possibleAction = false;
    if(action == "UP") {
        if (Pilot::heading == Pilot::Dir::UP) {
            if(obstacleFront){
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if (Pilot::heading == Pilot::Dir::LEFT) {
            if(obstacleRight){
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::RIGHT) {
            if(obstacleLeft){
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::LEFT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::DOWN) {
            if(obstacleBack) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }

    } else if (action == "LEFT"){
        if(Pilot::heading == Pilot::Dir::UP){
            if(obstacleLeft) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::LEFT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::LEFT){
            if(obstacleFront) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::DOWN) {
            if(obstacleRight) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::RIGHT){
            if(obstacleBack) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }

    } else if (action == "RIGHT"){
        if(Pilot::heading == Pilot::Dir::UP){
            if(obstacleRight) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::RIGHT){
            if(obstacleFront) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::DOWN){
            if(obstacleLeft) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::LEFT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::LEFT){
            if(obstacleBack) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }

    } else if (action == "DOWN"){
        if(Pilot::heading == Pilot::Dir::RIGHT){
            if(obstacleRight) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::LEFT){
            if(obstacleLeft) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::LEFT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::DOWN){
            if(obstacleFront) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
        if(Pilot::heading == Pilot::Dir::UP){
            if(obstacleBack) {
                Pilot::possibleAction = false;
            } else {
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::RIGHT);
                Pilot::commands.push(Pilot::Commands::FORWARD);
                Pilot::possibleAction = true;
            }
        }
    }
}


void Pilot::sendMessage(const std::string& message){
    ROS_INFO("SENT [%s]", message.c_str());
    //Send message
    std_msgs::String str;
    std::stringstream ss;
    ss << message;
    str.data = ss.str();
    Pilot::commandCompletedPublisher.publish(str);

}


void Pilot::irFrontCallback(const sensor_msgs::Range::ConstPtr& msg){
    // If obstacle at 0.5 m or less, true
    obstacleFront = msg->range < 0.5;
}

void Pilot::irLeftCallback(const sensor_msgs::Range::ConstPtr& msg){
    // If obstacle at 0.5 m or less, true
    obstacleLeft = msg->range < 0.5;
}

void Pilot::irRightCallback(const sensor_msgs::Range::ConstPtr& msg){
    // If obstacle at 0.5 m or less, true
    obstacleRight = msg->range < 0.5;
}

void Pilot::irBackCallback(const sensor_msgs::Range::ConstPtr& msg){
    // If obstacle at 0.5 m or less, true
    obstacleBack = msg->range < 0.5;
}