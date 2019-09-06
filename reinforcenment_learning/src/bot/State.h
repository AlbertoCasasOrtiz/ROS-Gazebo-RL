//
// Created by alberto on 6/09/19.
//

#ifndef SRC_STATE_H
#define SRC_STATE_H


#include "../utils/Point.h"

class State {
public:
    /// Position of the robot.
    Point p = Point(0, 0);

    /// Operator of equality of states.
    /// \param s Another state.
    /// \return true if the states are the same, false otherwise.
    bool operator == (State const &s);

    /// Operator of inequality of states.
    /// \param s Another state.
    /// \return true if the states are different, false otherwise.
    bool operator != (State const &s);

    /// Operator of lessority of states.
    /// \param s Another state.
    /// \return true if the states are less than other, false otherwise.
    bool operator < (State const &s) const;

    /// Get string representation of a state.
    /// \return String representation of a state.
    std::string toString();

    /// Constructor of State.
    /// \param x Position x of the robot.
    /// \param y Position y of the robot.
    State(int x, int y);
};


#endif //SRC_STATE_H
