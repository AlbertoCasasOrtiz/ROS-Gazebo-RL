//
// Created by alberto on 6/09/19.
//

#ifndef SRC_QLEARNING_H
#define SRC_QLEARNING_H


#include <ros/ros.h>
#include "bot/Bot.h"
#include "bot/Table.h"
#include "bot/Actions.h"
#include <std_msgs/String.h>
#include <mutex>
#include <condition_variable>

class QLearning {
public:
    /// Constructor of QLearning.
    QLearning(int argc, char **argv);

    /// Get reward when goal achieved.
    /// \return Reward.
    float getReward(State state);

    /// Ending condition of an episode.
    bool endCondition();
private:
    // Callback functions
    /// Receives and processes status from the pilot topic.
    /// \param msgs Message containing the status.
    void commanderCallback(const std_msgs::String::ConstPtr &msg);

    /**
     * Send a message to the pilot.
     * @param action Action in the message.
     */
    void sendMessage(Actions::Action action);
    void sendMessage(std::string string);
    /// Algorithm learning rate.
    float alpha = 0.0;
    /// Probability of taking a random action.
    float epsilon = 0.0;
    /// Algorithm propagation rate.
    float gamma = 0.0;
    // Influence of elegibility traces.
    float lambda = 0.0;

    /// Initial state of the robot.
    State initialState = State(0, 0);;
    /// Goal state of the robot.
    State goalState = State(0, 0);

    /// Inner representation of the robot.
    Bot bot = Bot(State(0, 0));

    /// Subscribe to pilot to receive robot status.
    ros::Subscriber commanderSubscriber;

    /// Publish commands to pilot
    ros::Publisher commandPublisher;

    //TODO documentation.
    State sP = State(0, 0);

    Actions::Action a;
    Actions::Action aP;
    Actions::Action aS;
    float delta = 0.0;
    bool newEpisode;
    float reward;
};


#endif //SRC_QLEARNING_H
