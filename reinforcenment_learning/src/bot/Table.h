//
// Created by alberto on 6/09/19.
//

#ifndef SRC_TABLE_H
#define SRC_TABLE_H

#include <vector>
#include <map>
#include "State.h"

class Table {
private:
    /// Table that stores the probability of taking an action. [x][y][a]
    std::map<State,std::vector<float>> tableQ;
    /// Table that stores elegibility traces of an execution. [x][y][a]
    std::map<State,std::vector<float>> tableE;

public:
    /// Initialize table Q to 0.
    void initializeTableQ(State initial);
    /// Initialize table E to 0.
    void initializeTableE(State initial);

    /// Update a value in the table Q.
    /// \param p Position of the robot.
    /// \param a Action taken by the robot.
    /// \param value Value added to the table Q cell.
    void updateValueQ(State p, int a, float value);
    /// Update a value in the table E.
    /// \param p Position of the robot.
    /// \param a Action taken by the robot.
    /// \param value Value added to the table Q cell.
    void updateValueE(State p, int a, float value);

    /// Get a value in the table Q.
    /// \param p Position of the robot.
    /// \param a Action taken by the robot.
    float getValueQ(State p, int a);

    /// Get a value in the table E.
    /// \param p Position of the robot.
    /// \param a Action taken by the robot.
    float getValueE(State p, int a);

    /// Get sizes of all dimensions of the table Q.
    /// \return Vector with sizes of all dimensions of the table Q.
    std::vector<int> getSizesTableQ();

    /// Get sizes of all dimensions of the table E.
    /// \return Vector with sizes of all dimensions of the table E.
    std::vector<int> getSizesTableE();



};


#endif //SRC_TABLE_H
