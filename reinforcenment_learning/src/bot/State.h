//
// Created by alberto on 6/09/19.
//

#ifndef SRC_STATE_H
#define SRC_STATE_H


class State {
public:
    /// Coordenate X in the grid.
    int x;
    /// Coordenate Y in the grid.
    int y;

    /// Operator of equality of states.
    /// \param s Another state.
    /// \return true if the states are the same, false otherwise.
    bool operator == (State const &s);

    /// Operator of inequality of states.
    /// \param s Another state.
    /// \return true if the states are different, false otherwise.
    bool operator != (State const &s);
};


#endif //SRC_STATE_H
