//
// Created by alberto on 6/09/19.
//

#include "Bot.h"

Bot::Bot(State currentState){
    Bot::currentState = currentState;
    // TODO set in qTable sizes of arrays. Maybe change to map of points that stores arrays of actions? 
    Bot::qTable.initializeTableQ();
    Bot::qTable.initializeTableE();
}
