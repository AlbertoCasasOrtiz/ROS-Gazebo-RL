//
// Created by alberto on 6/09/19.
//

#ifndef SRC_PILOT_H
#define SRC_PILOT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <sensor_msgs/Range.h>

class Pilot {
public:
    /// List of commands of the robot.
    enum class Commands {
        FORWARD, LEFT, RIGHT, STOP
    };

    /// Directions that the robot can head.
    enum class Dir {
        UP, DOWN, LEFT, RIGHT
    };

    /// Constructor of Pilot.
    /// \param argc Arguments from main.
    /// \param argv Arguments from main.
    Pilot(int argc, char **argv);

private:
    /// Send message to the robot. 1 -> Not ended, Not possible, 2 -> Ended, Nor Possible, 3 -> Ended and possible.
    /// \param message  Message to send.
    void sendMessage(const std::string& message);

    // Callback functions
    /// Receives and processes commands from a topic.
    /// \param msgs Message containing the command.
    void commanderCallback(const std_msgs::String::ConstPtr &msg);
    /// Receives and processes commands from /odom.
    /// \param msg Message containing information.
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    /// Callback function for IR front sensor.
    /// \param msg Message from IR front sensor.
    void irFrontCallback(const sensor_msgs::Range::ConstPtr& msg);
    /// Callback function for IR left sensor.
    /// \param msg Message from IR left sensor.
    void irLeftCallback(const sensor_msgs::Range::ConstPtr& msg);
    /// Callback function for IR right sensor.
    /// \param msg Message from IR right sensor.
    void irRightCallback(const sensor_msgs::Range::ConstPtr& msg);
    /// Callback function for IR back sensor.
    /// \param msg Message from IR back sensor.
    void irBackCallback(const sensor_msgs::Range::ConstPtr& msg);


    // Actions
    /// Execute the command FORWARD..
    /// \param velocity Linear velocity.
    void goForward(const nav_msgs::Odometry::ConstPtr &msg, float velocity);

    /// Execute the command LEFT.
    /// \param velocity Angular velocity of turn.
    void turnLeft(const nav_msgs::Odometry::ConstPtr &msg, float velocity);

    /// Execute the command RIGHT.
    /// \param velocity Angular velocity of turn.
    void turnRight(const nav_msgs::Odometry::ConstPtr &msg, float velocity);

    /// Execute the command STOP.
    void stop();

    //Helpers
    /// Update heading variable when turning to a direction.
    /// \param command Command sent to the robot.
    void updateHeading(Commands command);
    /// Parse a string representation of a command and return the command.
    /// \param command String representation of a command.
    /// \return Command represented.
    static Commands parseCommand(const std::string& command);
    /// Parse and action and convert it into a command.
    /// \param action Action to be parsed.
    /// \return Correspondent command.
    void parseAction(const std::string& action);

    //Publishers and Subscribers
    /// Subscribe to topic that send commands to the robot.
    ros::Subscriber commandSubscriber;
    /// Publish commands to the robot.
    ros::Publisher cmdVelPublisher;
    /// Subscribe to topic /odom.
    ros::Subscriber odomSubscriber;
    /// Publish if a command has been completed.
    ros::Publisher commandCompletedPublisher;
    /// Subscriber to front ir.
    ros::Subscriber rangeSubscriberFront;
    /// Subscriber to right ir.
    ros::Subscriber rangeSubscriberRight;
    /// Subscriber to left ir.
    ros::Subscriber rangeSubscriberLeft;
    /// Subscriber to back ir.
    ros::Subscriber rangeSubscriberBack;

    //Variables
    /// Queue of upcoming commands.
    std::queue<Commands> commands;
    /// Direction where the robot is heading.
    Dir heading;
    /// Speeds of the robot.
    float robotSpeed, robotAngularSpeed;
    /// Memory from last position and angle before start moving.
    float posX, posY, turnZ;
    /// Flag to execute commands just once.
    bool flag_once;
    /// Flag to notify the robot tha it has ended of moving.
    bool flag_notified;
    /// Length of stepts taken by the robot.
    float stepDistance;

    /// True if the action is possible.
    bool possibleAction;

    /// Send first command.
    bool firstTime;

    /// True if obstacle in a direction.
    bool obstacleLeft, obstacleFront, obstacleBack, obstacleRight;
};

#endif //SRC_PILOT_H
