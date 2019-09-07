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
public:
    /// Table that stores values for pairs state action. [x][y][a]
    std::map<State,std::vector<float>> table;

    /// Initialize table to 0.
    void initializeTable(State initial);

    /// Update a value in the table.
    /// \param state Position of the robot.
    /// \param a Action taken by the robot.
    /// \param value Value added to the table cell.
    void setValue(State state, int a, float value);

    /// Get a value in the table.
    /// \param p Position of the robot.
    /// \param a Action taken by the robot.
    float getValue(State p, int a);

    /// Get sizes of all dimensions of the table.
    /// \return Vector with sizes of all dimensions of the table.
    std::vector<int> getSizesTable();

    /// Add state to the table.
    /// \param state A new state.
    void addState(State state);

    /// Print table into file.
    void printTable(std::string name);
};


#endif //SRC_TABLE_H
