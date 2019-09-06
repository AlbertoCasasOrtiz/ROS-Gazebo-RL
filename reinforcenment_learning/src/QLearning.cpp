//
// Created by alberto on 6/09/19.
//

#include "QLearning.h"
#include "bot/Actions.h"
#include <ros/ros.h>

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
    QLearning::bot.tableQ.initializeTable(QLearning::bot.currentState);

    for(int i = 0; i < QLearning::numEpisodes; i++){
        // Initialize table E.
        QLearning::bot.tableE.initializeTable(QLearning::bot.currentState);

        // Initialize S and A.
        Actions::Action a = Actions::getAction(0);
        ROS_INFO("a [%s]", Actions::parseAction(a).c_str());
        QLearning::bot.currentState = QLearning::initialState;

        while(!endCondition()) {
            sP = Actions::takeAction(&(QLearning::bot), a);
            ROS_INFO("sP [(%i, %i)]", sP.p.x, sP.p.y);

            // Wait until robot_ends notified.
            if(flagRobotEnded && flagPossibleAction) {
                // Observe reward of sP
                float reward = QLearning::getReward(sP);
                ROS_INFO("Reward [%f]", reward);

                // Get action from eGreedy.
                Actions::Action aP = Actions::eGreedy(QLearning::bot, QLearning::epsilon);
                ROS_INFO("aP [%s]", Actions::parseAction(aP).c_str());
                // Get best action.
                Actions::Action aS = Actions::bestAction(QLearning::bot);
                ROS_INFO("aS [%s]", Actions::parseAction(aS).c_str());

                ROS_INFO("TableQ size [%d]", QLearning::bot.tableQ.getSizesTable().at(0));
                ROS_INFO("State [%s]", sP.toString().c_str());

                // Update delta.
                float QSpA = QLearning::bot.tableQ.getValue(sP, Actions::getPosition(aS));
                ROS_INFO("QSA [%f]", QSpA);
                float QSpAs = QLearning::bot.tableQ.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                ROS_INFO("QSpAs [%f]", QSpAs);
                delta = reward + QLearning::gamma * QSpA - QSpAs;
                ROS_INFO("delta [%f]", delta);

                // Choose strategy of update traces.
                float ESA = QLearning::bot.tableE.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                QLearning::bot.tableE.updateValue(QLearning::bot.currentState, Actions::getPosition(a), ESA + 1);

                ROS_INFO("HOLA");

                // Update tables-
                for (int i = 0; i < QLearning::bot.tableQ.getSizesTable().at(0); i++) {
                    // Update table Q.
                    float newQ =
                            QLearning::alpha * delta *
                            QLearning::bot.tableE.getValue(QLearning::bot.currentState, a);
                    QLearning::bot.tableQ.updateValue(QLearning::bot.currentState, a, newQ);
                    //Update table E.
                    if (a == aP) {
                        float newE = QLearning::lambda * QLearning::gamma *
                                     QLearning::bot.tableE.getValue(QLearning::bot.currentState, a);
                        QLearning::bot.tableE.updateValue(QLearning::bot.currentState, a, newE);
                    } else {
                        QLearning::bot.tableE.updateValue(QLearning::bot.currentState, a, 0);
                    }
                    QLearning::bot.currentState = sP;
                    a = aP;
                }
            } else {
                //If action not possible mark probability of take it as minus infinity.
                QLearning::bot.tableQ.updateValue(QLearning::bot.currentState, a, std::numeric_limits<int>::min());
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
