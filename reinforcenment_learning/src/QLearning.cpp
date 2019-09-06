//
// Created by alberto on 6/09/19.
//

#include "QLearning.h"
#include "bot/Actions.h"

QLearning::QLearning() {
    // TODO Initialize values here.
    QLearning::numEpisodes = 100;
    QLearning::alpha = 0.7;
    QLearning::epsilon = 0.2;
    QLearning::gamma = 0.2;

}

void QLearning::execute() {
    State sP;
    float delta = 0.0;

    // Initialize table Q.;
    QLearning::bot.qTable.initializeTableQ();

    for(int i = 0; i < QLearning::numEpisodes; i++){
        // Initialize table E.
        QLearning::bot.qTable.initializeTableE();

        // Initialize S and A.
        Actions::Action a = Actions::getAction(0);
        QLearning::bot.currentState = QLearning::initialState;

        while(!endCondition()) {
            // TODO Tomar acción y guardar en sP el nuevo estado.
            // sP = Resultado de tomar acción.
            // TODO Esperar a que acción sea tomada.

            //TODO Calculate reward.
            float reward;

            // Get action from eGreedy.
            Actions::Action aP = Actions::eGreedy(QLearning::bot, QLearning::epsilon);
            // Get best action.
            Actions::Action aS = Actions::bestAction(QLearning::bot);

            // Update delta.
            delta = reward + QLearning::gamma * QLearning::bot.qTable.getValueQ(sP.x, sP.y, Actions::getPosition(aS)) - QLearning::bot.qTable.getValueQ(QLearning::bot.currentState.x, QLearning::bot.currentState.y, Actions::getPosition(a));

            //Choose strategy of update traces.
            QLearning::bot.qTable.updateValueE(QLearning::bot.currentState.x, QLearning::bot.currentState.y, Actions::getPosition(a), QLearning::bot.qTable.getValueE(QLearning::bot.currentState.x, QLearning::bot.currentState.y, Actions::getPosition(a)));

            // TODO Incluir bucle for.
        }

        QLearning::bot.currentState = sP;
    }
}

float QLearning::getReward() {
    if(QLearning::bot.currentState == QLearning::goalState){
        return 100;
    } else return -0.1;
}

bool QLearning::endCondition() {
    return QLearning::bot.currentState != QLearning::goalState;
}
