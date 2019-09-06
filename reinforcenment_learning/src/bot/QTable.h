//
// Created by alberto on 6/09/19.
//

#ifndef SRC_QTABLE_H
#define SRC_QTABLE_H

#include <vector>

class QTable {
private:
    /// Table that stores the probability of taking an action. [x][y][a]
    std::vector<std::vector<std::vector<float>>> tableQ;
    /// Table that stores elegibility traces of an execution. [x][y][a]
    std::vector<std::vector<std::vector<float>>> tableE;
public:
    /// Initialize table Q to 0.
    void initializeTableQ();
    /// Initialize table E to 0.
    void initializeTableE();

    /// Update a value in the table Q.
    /// \param x Coordinate X of the position of the robot.
    /// \param y Coordinate Y of the position of the robot.
    /// \param a Action taken by the robot.
    /// \param value Value added to the table Q cell.
    void updateValueQ(int x, int y, int a, float value);
    /// Update a value in the table E.
    /// \param x Coordinate X of the position of the robot.
    /// \param y Coordinate Y of the position of the robot.
    /// \param a Action taken by the robot.
    /// \param value Value added to the table Q cell.
    void updateValueE(int x, int y, int a, float value);

    /// Get a value in the table Q.
    /// \param x Coordinate X of the position of the robot.
    /// \param y Coordinate Y of the position of the robot.
    /// \param a Action taken by the robot.
    float getValueQ(int x, int y, int a);

    /// Get a value in the table E.
    /// \param x Coordinate X of the position of the robot.
    /// \param y Coordinate Y of the position of the robot.
    /// \param a Action taken by the robot.
    float getValueE(int x, int y, int a);

    /// Get sizes of all dimensions of the table Q.
    /// \return Vector with sizes of all dimensions of the table Q.
    std::vector<int> getSizesTableQ();

    /// Get sizes of all dimensions of the table E.
    /// \return Vector with sizes of all dimensions of the table E.
    std::vector<int> getSizesTableE();



};


#endif //SRC_QTABLE_H
