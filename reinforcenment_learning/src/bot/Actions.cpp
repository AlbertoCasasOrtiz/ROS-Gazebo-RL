#include <utility>

//
// Created by alberto on 6/09/19.
//

#include <rosconsole/macros_generated.h>
#include "Actions.h"
#include <ros/ros.h>
#include <random>
#include <iostream>
#include "../utils/RandomGenerator.h"
#include <limits>

std::string Actions::toString(Actions::Action action) {
    switch(action){
        case Actions::Action::UP:
            return "UP";
        case Actions::Action::LEFT:
            return "LEFT;";
        case Actions::Action::RIGHT:
            return "RIGHT";
        case Actions::Action::DOWN:
            return "DOWN";
        default:
            return "";
    }
}

Actions::Action Actions::getAction(int i) {
    switch(i){
        case 0:
            return Actions::Action::UP;
        case 1:
            return Actions::Action::LEFT;
        case 2:
            return Actions::Action::RIGHT;
        case 3:
            return Actions::Action::DOWN;
        default:
            ROS_ERROR("Wrong action number [%i].", i);
            return Actions::Action::UP;

    }
}

int Actions::getPosition(Actions::Action action) {
    switch(action){
        case Actions::Action::UP:
            return 0;
        case Actions::Action::LEFT:
            return 1;
        case Actions::Action::RIGHT:
            return 2;
        case Actions::Action::DOWN:
            return 3;
        default:
            return -1;
    }
}

Actions::Action Actions::eGreedy(Bot bot, float epsilon) {
    Action action;
    // Generate random number between 0 and 1.
    float rand = RandomGenerator::getFloat(0, 1);

    if(rand < epsilon){
        // Return random action.
        int n = RandomGenerator::getInt(0, 4);
        action = Actions::getAction(n);
    } else {
        // Return best action.
        action = Actions::bestAction(std::move(bot));
    }

    return action;
}

Actions::Action Actions::bestAction(Bot bot) {
    Actions::Action bestAction = Actions::Action::UP;
    float max = std::numeric_limits<int>::min();
    for(int i = 0; i < Actions::size; i++) {
        float actionValue = bot.tableQ.getValue(bot.currentState, i);
        if (actionValue > max) {
            bestAction = Actions::getAction(i);
            max = actionValue;
        }
    }
    return bestAction;
}

State Actions::takeAction(Bot *bot, Actions::Action action) {
    // Create new state that is the consequence of taking the action.
    State state = State(bot->currentState.p.x, bot->currentState.p.y);
    switch(action){
        case Actions::Action::UP:
            state.p.y++;
            break;
        case Actions::Action::LEFT:
            state.p.x--;
            break;
        case Actions::Action::RIGHT:
            state.p.x++;
            break;
        case Actions::Action::DOWN:
            state.p.y--;
            break;
    }

    // Add the new state to the tables.
    bot->tableE.addState(state);
    bot->tableQ.addState(state);
    return state;
}
