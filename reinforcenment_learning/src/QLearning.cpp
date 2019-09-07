//
// Created by alberto on 6/09/19.
//

#include "QLearning.h"
#include "bot/Actions.h"
#include <ros/ros.h>
#include <thread>
#include "std_srvs/Empty.h""

QLearning::QLearning(int argc, char **argv) {
    // TODO Initialize values here.
    QLearning::numEpisodes = 100;
    QLearning::alpha = 0.7;
    QLearning::epsilon = 0.2;
    QLearning::gamma = 0.2;
    QLearning::lambda = 0.2;

    QLearning::initialState = State(0, 0);
    QLearning::goalState = State(2, 2);
    QLearning::bot = Bot(QLearning::initialState);

    QLearning::newEpisode = false;

    // ROS starts here.
    ros::init(argc, argv, "commander");

    // Nodehandler for handle other nodes.
    ros::NodeHandle nh;

    // Initialize subscribers and publishers
    // Subscribe of commander for getting actions
    QLearning::commanderSubscriber = nh.subscribe("/commander", 10, &QLearning::commanderCallback, this);
    // Publish commands to pilot.
    QLearning::commandPublisher = nh.advertise<std_msgs::String>("/pilot", 10);
    // Execute algorithm.

    ros::spin();
}


float QLearning::getReward(State state) {
    if(state == goalState){
        return 100;
    } else return -0.1;
}

bool QLearning::endCondition() {
    return QLearning::bot.currentState == QLearning::goalState;
}

void QLearning::commanderCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("RECEIVED: [%s]", msg->data.c_str());
    if(std::string(msg->data) == "init") {
        ROS_INFO("INITIALIZING..");
        // Initialize table Q.;
        QLearning::bot.tableQ.initializeTable(QLearning::bot.currentState);
        QLearning::newEpisode = true;

        sendMessage("algorithm_initialized");
    } else {

        if (newEpisode) {
            //TODO restart simulation.

            // Initialize table E.
            QLearning::bot.tableE.initializeTable(QLearning::bot.currentState);

            // Initialize S and A.
            QLearning::bot.currentState = QLearning::initialState;
            a = Actions::getAction(4);
            newEpisode = false;
        } else {
            if (!endCondition()) {
                if (msg->data == "not possible" || msg->data == "next movement") {
                    // If action not possible (there is an obstacle), put it as not possible in Q table with minimum value.
                    if (msg->data == "not possible") {
                        QLearning::bot.tableQ.setValue(sP, aP, std::numeric_limits<int>::min());
                    }

                    // Take action a.
                    sP = Actions::takeAction(&(QLearning::bot), a);
                    // Observe reward of sP
                    reward = QLearning::getReward(sP);

                    // Get action from eGreedy.
                    aP = Actions::eGreedy(sP, QLearning::epsilon);
                    sendMessage(aP);
                }
                if (msg->data == "possible") {
                    // Get best action.
                    aS = Actions::bestAction(QLearning::bot);
                    ROS_INFO("Action; [%s]", Actions::toString(aP).c_str());
                    ROS_INFO("State: [%i][%i]", sP.p.x, sP.p.y);

                    // Update delta.
                    float QSA = QLearning::bot.tableQ.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                    float QSpAs = QLearning::bot.tableQ.getValue(sP, Actions::getPosition(aP));
                    delta = reward + QLearning::gamma * QSpAs - QSA;

                    // Choose strategy of update traces.
                    float ESA = QLearning::bot.tableE.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                    QLearning::bot.tableE.setValue(QLearning::bot.currentState, Actions::getPosition(a), ESA + 1);

                    // Update tables-
                    for (int i = 0; i < QLearning::bot.tableQ.getSizesTable().at(0); i++) {
                        // Update table Q.
                        float Q = QLearning::bot.tableQ.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                        float newQ =
                                QLearning::alpha * delta *
                                QLearning::bot.tableE.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                        QLearning::bot.tableQ.setValue(QLearning::bot.currentState, a, Q + newQ);
                        //Update table E.
                        if (aS == aP) {
                            float newE = QLearning::lambda * QLearning::gamma *
                                         QLearning::bot.tableE.getValue(QLearning::bot.currentState,
                                                                        Actions::getPosition(a));
                            QLearning::bot.tableE.setValue(QLearning::bot.currentState, Actions::getPosition(a), newE);
                        } else {
                            QLearning::bot.tableE.setValue(QLearning::bot.currentState, Actions::getPosition(a), 0);
                        }

                    }
                    // Update robot status and a.
                    a = aP;
                    QLearning::bot.currentState = sP;

                    //ROS_INFO("Waiting robot status.");
                    ROS_INFO(" ");
                }
            } else {
                QLearning::sendMessage(Actions::Action::STOP);
                newEpisode = true;
                std_srvs::Empty resetWorldSrv;
                ros::service::call("/gazebo/reset_world", resetWorldSrv);
                ros::service::call("/gazebo/reset_simulation", resetWorldSrv);
            }
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

void QLearning::sendMessage(std::string string) {
    ROS_INFO("SENT [%s]", string.c_str());
    //Send message
    std_msgs::String str;
    std::stringstream ss;

    ss << string;
    str.data = ss.str();
    commandPublisher.publish(str);
}
