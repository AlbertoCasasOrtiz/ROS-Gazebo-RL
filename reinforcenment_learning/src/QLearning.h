//
// Created by alberto on 6/09/19.
//

#ifndef SRC_QLEARNING_H
#define SRC_QLEARNING_H


#include "bot/Bot.h"
#include "bot/Table.h"

class QLearning {
public:
    /// Constructor of QLearning.
    QLearning(int argc, char **argv);

    //TODO
    /// \param other
    QLearning(QLearning & other);

    /// Get reward when goal achieved.
    /// \return Reward.
    float getReward(State state);

    /// Execution of the algorithm Q-Learning.
    void execute();

    /// Ending condition of an episode.
    bool endCondition();
private:
    /// Max number of episodes of the algorithm-
    int numEpisodes = 0;
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

    /// Indicates if an action is possible.
    bool flagPossibleAction = false;

    /// Indicates if the robot has finished it movement.
    bool flagRobotEnded = false;

};


#endif //SRC_QLEARNING_H
