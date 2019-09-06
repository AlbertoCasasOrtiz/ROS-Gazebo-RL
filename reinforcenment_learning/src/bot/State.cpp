//
// Created by alberto on 6/09/19.
//

#include <string>
#include "State.h"

bool State::operator==(State const &s) {
    return p.x == s.p.x && p.y == s.p.y;
}

bool State::operator!=(State const &s) {
    return p.x != s.p.x || p.y != s.p.y;
}

State::State(int x, int y) {
    State::p = Point(x, y);
}

bool State::operator<(State const &s) const {
    std::string stringP1, stringP2;
    stringP1 = std::to_string(p.x) + std::to_string(p.y);
    stringP2 = std::to_string(s.p.x) + std::to_string(s.p.y);

    return stringP1 < stringP2;
}