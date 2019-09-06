//
// Created by alberto on 6/09/19.
//

#ifndef SRC_BOT_H
#define SRC_BOT_H


#include "QTable.h"
#include "State.h"

class Bot {
public:
    explicit Bot(State currentState);

public:
    /// Current state of the robot.
    State currentState;
    /// QTable with learned values.
    QTable qTable;
};


#endif //SRC_BOT_H
