//
// Created by alberto on 6/09/19.
//

#include "QLearning.h"
#include "bot/Actions.h"

QLearning::QLearning(int argc, char **argv) {
    // TODO Initialize values here.
    QLearning::numEpisodes = 100;
    QLearning::alpha = 0.7;
    QLearning::epsilon = 0.2;
    QLearning::gamma = 0.2;
    QLearning::lambda = 0.2;

    QLearning::initialState = State(0, 0);
    QLearning::goalState = State(0, 0);
    QLearning::bot = Bot(QLearning::initialState);

}

void QLearning::execute() {
    State sP = State(0, 0);
    float delta = 0.0;

    // Initialize table Q.;
    QLearning::bot.tables.initializeTableQ(QLearning::bot.currentState);

    for(int i = 0; i < QLearning::numEpisodes; i++){
        // Initialize table E.
        QLearning::bot.tables.initializeTableE(QLearning::bot.currentState);

        // Initialize S and A.
        Actions::Action a = Actions::getAction(0);
        QLearning::bot.currentState = QLearning::initialState;

        while(!endCondition()) {
            sP = Actions::takeAction(QLearning::bot, a);
            // TODO Esperar a que acción sea tomada por robot real.

            // Observe reward of sP
            float reward = QLearning::getReward(sP);

            // Get action from eGreedy.
            Actions::Action aP = Actions::eGreedy(QLearning::bot, QLearning::epsilon);
            // Get best action.
            Actions::Action aS = Actions::bestAction(QLearning::bot);

            // Update delta.
            delta = reward + QLearning::gamma * QLearning::bot.tables.getValueQ(sP, Actions::getPosition(aS)) - QLearning::bot.tables.getValueQ(QLearning::bot.currentState, Actions::getPosition(a));

            // Choose strategy of update traces.
            QLearning::bot.tables.updateValueE(QLearning::bot.currentState, Actions::getPosition(a), QLearning::bot.tables.getValueE(QLearning::bot.currentState, Actions::getPosition(a)));

            // Update tables-
            for(int i = 0; i < QLearning::bot.tables.getSizesTableE().at(0); i++){
                // Update table Q.
                float newQ = QLearning::alpha * delta * QLearning::bot.tables.getValueE(QLearning::bot.currentState, a);
                QLearning::bot.tables.updateValueQ(QLearning::bot.currentState, a,newQ);
                //Update table E.
                if(a == aP){
                    float newE = QLearning::lambda * QLearning::gamma * QLearning::bot.tables.getValueE(QLearning::bot.currentState, a);
                    QLearning::bot.tables.updateValueE(QLearning::bot.currentState, a, newE);
                } else {
                    QLearning::bot.tables.updateValueE(QLearning::bot.currentState, a, 0);
                }
                QLearning::bot.currentState = sP;
                a = aP;
            }
        }

        QLearning::bot.currentState = sP;
    }
}

float QLearning::getReward(State state) {
    if(state == state){
        return 100;
    } else return -0.1;
}

bool QLearning::endCondition() {
    return QLearning::bot.currentState != QLearning::goalState;
}
