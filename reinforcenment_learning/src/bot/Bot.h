//
// Created by alberto on 6/09/19.
//

#ifndef SRC_BOT_H
#define SRC_BOT_H


#include "Table.h"
#include "State.h"

class Bot {
public:
    /// Constructor of Bot.
    /// \param currentState Current state of the bot.
    Bot(State currentState);
public:
    /// Current state of the robot.
    State currentState = State(0, 0);
    /// QTable with learned values.
    Table tables;
};


#endif //SRC_BOT_H
