//
// Created by alberto on 6/09/19.
//

#ifndef SRC_ACTIONS_H
#define SRC_ACTIONS_H

#include <string>
#include "Bot.h"

class Actions {
public:
    /// Number of actions.
    const static int size = 4;
    /// Actions that the robot can execute.
    enum Action{UP, DOWN, LEFT, RIGHT, STOP};

    /// Parse a action in string representation.
    /// \param action String representation of a action.
    /// \return action represented as string.
    static std::string toString(Action action);

    /// Get ith action.
    /// \param i Number of the action.
    /// \return ith action.
    static Actions::Action getAction(int i);

    /// Get position of a action.
    /// \param action Action to get position.
    /// \return Position of the ation.
    static int getPosition(Action action);


    /// Epsilon greedy method. Take a random action or the best depending of prob epsilon.
    /// \param bot Bot with its current state and tables.
    /// \param epsilon Probability of taking a random action.
    /// \return Selected action by the method.
    static Actions::Action eGreedy(Bot bot, float epsilon);

    /// Get the best action that the bot can take.
    /// \param bot Bot with its current state and tables.
    /// \return Selected action by the metod.
    static Actions::Action bestAction(Bot bot);

    /// Take and action and return state.
    /// \param bot Bot that takes the action.
    /// \param action Action to take.
    /// \return State result of take an action.
    static State takeAction(Bot *bot, Action action);
};


#endif //SRC_ACTIONS_H
