//
// Created by alberto on 6/09/19.
//

#ifndef SRC_QLEARNING_H
#define SRC_QLEARNING_H


#include "state/State.h"
#include "bot/Bot.h"
#include "bot/QTable.h"

class QLearning {
public:
    /// Constructor of QLearning.
    QLearning();

    /// Get reward when goal achieved.
    /// \return Reward.
    float getReward();

    /// Execution of the algorithm Q-Learning.
    void execute();

    /// Ending condition of an episode.
    bool endCondition();
private:
    /// Max number of episodes of the algorithm-
    int numEpisodes;
    /// Algorithm learning rate.
    float alpha;
    /// Probability of taking a random action.
    float epsilon;
    /// Algorithm propagation rate.
    float gamma;

    /// Initial state of the robot.
    State initialState;
    /// Goal state of the robot.
    State goalState;

    /// Inner representation of the robot.
    Bot bot;

};


#endif //SRC_QLEARNING_H
