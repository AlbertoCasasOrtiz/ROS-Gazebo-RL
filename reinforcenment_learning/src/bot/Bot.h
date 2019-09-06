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
    /// Table with Q values.
    Table tableQ;
    /// Table with E valus.
    Table tableE;
};


#endif //SRC_BOT_H
