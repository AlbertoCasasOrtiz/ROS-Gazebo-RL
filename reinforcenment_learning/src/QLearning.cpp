//
// Created by alberto on 6/09/19.
//

#include "QLearning.h"
#include "bot/Actions.h"
#include <ros/ros.h>
#include <thread>
#include "std_srvs/Empty.h"
#include <math.h>

QLearning::QLearning(int argc, char **argv) {
    // TODO Initialize values here.
    QLearning::alphaini = 0.9;
    QLearning::tAlpha = 1.5;
    QLearning::epsilonini = 0.7;
    QLearning::tEpsilon = 1.5;
    QLearning::gamma = 0.5;
    QLearning::lambda = 0.5;

    QLearning::initialState = State(0, 0);
    QLearning::goalState = State(2, 2);
    QLearning::bot = Bot(QLearning::initialState);

    QLearning::episode = 0;

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
    } else return -0.01;
}

bool QLearning::endCondition() {
    return QLearning::bot.currentState == QLearning::goalState;
}

void QLearning::commanderCallback(const std_msgs::String::ConstPtr &msg) {
   // ROS_INFO("RECEIVED: [%s]", msg->data.c_str());
    if(std::string(msg->data) == "init") {
        ROS_INFO("GRETTINGS ALBERTO..");
        ROS_INFO("INITIALIZING..");
        // Initialize table Q.;
        QLearning::bot.tableQ.initializeTable(QLearning::bot.currentState);
        QLearning::newEpisode = true;

        sendMessage("algorithm_initialized");
    } else {

        if (newEpisode) {
            // Initialize table E.
            QLearning::bot.tableE.initializeTable(QLearning::bot.currentState);

            // Initialize S and A.
            QLearning::bot.currentState = QLearning::initialState;
            a = Actions::getAction(4);
            newEpisode = false;

            ROS_INFO("Epsilon [%f]:", epsilonValue());
            ROS_INFO("Alpha [%f]:", alphaValue());

        } else {
            if (!endCondition()) {
                if (msg->data == "not possible" || msg->data == "next movement") {
                    if(msg->data == "not possible" )
                        QLearning::bot.tableQ.setValue(sP, aP, -5);
                    // Take action a.
                    sP = Actions::takeAction(&(QLearning::bot), a);

                    // Get action from eGreedy.
                    aP = Actions::eGreedy(bot, sP, QLearning::epsilonValue());

                    if(sP != goalState)
                        sendMessage(aP);
                    else {
                        sendMessage("goal");
                        ROS_INFO("MISSION ACCOMPLISHED.");
                    }
                }
                if (msg->data == "possible") {
                    // Observe reward of sP
                    reward = QLearning::getReward(sP);
                    //ROS_INFO("REWARD [%f]", reward);

                    // Get best action.
                    aS = Actions::bestAction(bot, sP);
                    ROS_INFO("CURRENT STATE: [%i][%i]", sP.p.x, sP.p.y);
                    ROS_INFO("NEXT ACTION: [%s]", Actions::toString(aP).c_str());

                    // Update delta.
                    float QSA = QLearning::bot.tableQ.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                    float QSpAs = QLearning::bot.tableQ.getValue(sP, Actions::getPosition(aP));
                    delta = reward + QLearning::gamma * QSpAs - QSA;

                    // Choose strategy of update traces.
                    float ESA = QLearning::bot.tableE.getValue(QLearning::bot.currentState, Actions::getPosition(a));
                    QLearning::bot.tableE.setValue(QLearning::bot.currentState, Actions::getPosition(a), ESA + 1);

                    // Update tables-

                    for(std::map<State, std::vector<float>>::const_iterator it = QLearning::bot.tableQ.table.begin(); it != QLearning::bot.tableQ.table.end(); ++it) {
                        State aux = it->first;
                        for (int j = 0; j < Actions::size; j++) {
                            // Update table Q.
                            float Q = QLearning::bot.tableQ.getValue(aux, j);
                            //if(reward == 10) ROS_INFO("REWARD Q: [%f]", Q);
                            float newQ = QLearning::alphaValue() * delta * QLearning::bot.tableE.getValue(aux, j);
                            //if(reward == 10) ROS_INFO("REWARD newQ: [%f]", newQ);
                            QLearning::bot.tableQ.setValue(aux, j, Q + newQ);
                            //Update table E.
                            if (aS == aP) {
                                float newE = QLearning::lambda * QLearning::gamma *
                                             QLearning::bot.tableE.getValue(aux, j);
                                QLearning::bot.tableE.setValue(aux, j, newE);
                            } else {
                                QLearning::bot.tableE.setValue(aux, j, 0);
                            }
                        }

                    }
                    // Update robot status and a.
                    a = aP;
                    QLearning::bot.currentState = sP;

                    //ROS_INFO("Waiting robot status.");
                    ROS_INFO(" ");
                    QLearning::bot.tableQ.printTable("tableQ.txt");
                }
            } else {
                QLearning::episode++;
                QLearning::sendMessage(Actions::Action::STOP);
                QLearning::bot.tableQ.printTable("tableQ.txt");
                QLearning::bot.tableE.printTable("tableE.txt");
                newEpisode = true;
                // Restart Simulation.
                std_srvs::Empty resetWorldSrv;
                ros::service::call("/gazebo/reset_world", resetWorldSrv);
                ros::service::call("/gazebo/reset_simulation", resetWorldSrv);
            }
        }
    }
}

void QLearning::sendMessage(Actions::Action action) {
    //ROS_INFO("SENT [%s]", Actions::toString(action).c_str());
    //Send message
    std_msgs::String str;
    std::stringstream ss;

    ss << Actions::toString(action);
    str.data = ss.str();
    commandPublisher.publish(str);
}

void QLearning::sendMessage(const std::string& string) {
    //ROS_INFO("SENT [%s]", string.c_str());
    //Send message
    std_msgs::String str;
    std::stringstream ss;

    ss << string;
    str.data = ss.str();
    commandPublisher.publish(str);
}

float QLearning::epsilonValue() {
    float part = exp(-QLearning::tEpsilon/(QLearning::episode+1));
    return QLearning::epsilonini+(1-QLearning::epsilonini)*part;
}

float QLearning::alphaValue() {
    float part = exp(-QLearning::tAlpha/(QLearning::episode+1));
    return QLearning::alphaini-QLearning::alphaini*part;
}
