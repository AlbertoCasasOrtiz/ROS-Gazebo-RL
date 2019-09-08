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
    /// Constructor and destructor.
    /// Constructor of QLearning.
    QLearning(int argc, char **argv);

    /// Class methods.
    /// Get reward when goal achieved.
    /// \return Reward.
    float getReward(State state);
    /// Ending condition of an episode.
    bool endCondition();
private:
    // Publishers and subscribers.
    /// Subscribe to pilot to receive robot status.
    ros::Subscriber commanderSubscriber;
    /// Publish commands to pilot
    ros::Publisher commandPublisher;

    // Callback functions
    /// Receives and processes status from the pilot topic.
    /// \param msgs Message containing the status.
    void commanderCallback(const std_msgs::String::ConstPtr &msg);

    // IO Functions.
    /**
     * Send a message to the pilot.
     * @param action Action in the message.
     */
    void sendMessage(Actions::Action action);
    /**
     * Send a message to the pilot.
     * @param string Message in a string.
     */
    void sendMessage(const std::string& string);

    // Helpers.
    /// Calculate value of epsilon with a temperature function.
    /// \return Value of epsilon.
    float epsilonValue();
    /// Calculate value of alpha with a temperature function.
    /// \return Value of alpha.
    float alphaValue();

    // Class variables.
    /// Algorithm learning rate.
    float alphaini;
    float tAlpha;
    /// Probability of taking a random action.
    float epsilonini;
    float tEpsilon;
    /// Algorithm propagation rate.
    float gamma;
    /// Influence of elegibility traces.
    float lambda;
    /// Value calculated by algorithm to update tables.
    float delta;
    /// Number of episode.
    int episode;
    /// Initial state of the robot.
    State initialState = State(0, 0);;
    /// Goal state of the robot.
    State goalState = State(0, 0);
    /// Inner representation of the robot.
    Bot bot = Bot(State(0, 0));
    /// Next state after takin action.
    State sP = State(0, 0);
    /// Current action.
    Actions::Action a;
    /// Next action.
    Actions::Action aP;
    /// Best next action.
    Actions::Action aS;
    /// Indicate if new episode necessary.
    bool newEpisode;
    /// Reward of the state.
    float reward;
};


#endif //SRC_QLEARNING_H
