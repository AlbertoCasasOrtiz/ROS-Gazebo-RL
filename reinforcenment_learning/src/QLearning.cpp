//
// Created by alberto on 6/09/19.
//

#include "QLearning.h"
#include "bot/Actions.h"
#include <ros/ros.h>
#include <thread>

QLearning::QLearning(int argc, char **argv) {
    // TODO Initialize values here.
    QLearning::numEpisodes = 100;
    QLearning::alpha = 0.7;
    QLearning::epsilon = 0.7;
    QLearning::gamma = 0.2;
    QLearning::lambda = 0.2;

    QLearning::initialState = State(0, 0);
    QLearning::goalState = State(2, 2);
    QLearning::bot = Bot(QLearning::initialState);

    flag_pilot_ready = false;
    flag_initialize = true;
    flag_terminated_possible = true;

    // ROS starts here.
    ros::init(argc, argv, "commander");

    // Nodehandler for handle other nodes.
    ros::NodeHandle nh;

    // Initialize subscribers and publishers
    // Subscribe of commander for getting actions
    QLearning::commanderSubscriber = nh.subscribe("/commander", 1, &QLearning::commanderCallback, this);
    // Publish commands to pilot.
    QLearning::commandPublisher = nh.advertise<std_msgs::String>("/pilot", 10);
    // Execute algorithm.

    QLearning::execute();

    ros::spin();
}

void QLearning::execute() {
}

float QLearning::getReward(State state) {
    if(state == state){
        return 100;
    } else return -0.1;
}

bool QLearning::endCondition() {
    newEpisode = true;
    return QLearning::bot.currentState == QLearning::goalState;
}

void QLearning::commanderCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("RECEIVED: [%s]", msg->data.c_str());
    if(std::string(msg->data) == "init") {
        // Initialize table Q.;
        QLearning::bot.tableQ.initializeTable(QLearning::bot.currentState);
        flag_initialize = false;

    } if(newEpisode) {
        // Initialize table E.
        QLearning::bot.tableE.initializeTable(QLearning::bot.currentState);

        // Initialize S and A.
        QLearning::bot.currentState = QLearning::initialState;
        a = Actions::getAction(0);
        newEpisode = false;
    } else {
        if(!endCondition()) {
            if (std::string(msg->data) == "not possible" || std::string(msg->data) == "next movement") {

                // If action not possible (there is an obstacle), put it as not possible in Q table with minimum value.
                if(std::string(msg->data) == "not possible"){
                    QLearning::bot.tableQ.updateValue(sP, aP, std::numeric_limits<int>::min());
                }

                // Take action a.
                sP = Actions::takeAction(&(QLearning::bot), a);
                // Observe reward of sP
                float reward = QLearning::getReward(sP);

                // Get action from eGreedy.
                aP = Actions::eGreedy(sP, QLearning::epsilon);
                sendMessage(aP);
            } else if (std::string(msg->data) == "possible") {
                // Get best action.
                aS = Actions::bestAction(QLearning::bot);

                // Update delta.
                float QSpA = QLearning::bot.tableQ.getValue(sP, Actions::getPosition(aS));
                float QSpAs = QLearning::bot.tableQ.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                delta = reward + QLearning::gamma * QSpA - QSpAs;

                // Choose strategy of update traces.
                float ESA = QLearning::bot.tableE.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                QLearning::bot.tableE.updateValue(QLearning::bot.currentState, Actions::getPosition(a), ESA + 1);

                // Update tables-
                for (int i = 0; i < QLearning::bot.tableQ.getSizesTable().at(0); i++) {
                    // Update table Q.
                    float newQ =
                            QLearning::alpha * delta *
                            QLearning::bot.tableE.getValue(QLearning::bot.currentState, a);
                    QLearning::bot.tableQ.updateValue(QLearning::bot.currentState, a, newQ);
                    //Update table E.
                    if (aS == aP) {
                        float newE = QLearning::lambda * QLearning::gamma *
                                     QLearning::bot.tableE.getValue(QLearning::bot.currentState, a);
                        QLearning::bot.tableE.updateValue(QLearning::bot.currentState, a, newE);
                    } else {
                        QLearning::bot.tableE.updateValue(QLearning::bot.currentState, a, 0);
                    }

                }
                // Update robot status and a.
                a = aP;
                QLearning::bot.currentState = sP;

                //ROS_INFO("Waiting robot status.");
            }
        } else{
            newEpisode = true;
        }
    }
}

void QLearning::sendMessage(Actions::Action action) {
    ROS_INFO("SENT [%s]", Actions::toString(action).c_str());
    //Send message
    std_msgs::String str;
    std::stringstream ss;

    ss << Actions::toString(action);
    str.data = ss.str();
    commandPublisher.publish(str);
}
