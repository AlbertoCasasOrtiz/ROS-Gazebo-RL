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
    // Influence of elegibility traces.
    float lambda;

    /// Initial state of the robot.
    State initialState = State(0, 0);;
    /// Goal state of the robot.
    State goalState = State(0, 0);

    /// Inner representation of the robot.
    Bot bot = Bot(State(0, 0));

};


#endif //SRC_QLEARNING_H
