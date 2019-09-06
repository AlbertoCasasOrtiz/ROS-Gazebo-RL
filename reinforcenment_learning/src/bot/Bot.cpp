//
// Created by alberto on 6/09/19.
//

#include "Bot.h"

Bot::Bot(State currentState){
    Bot::currentState = currentState;
    Bot::tableE.initializeTable(currentState);
    Bot::tableQ.initializeTable(currentState);
}
