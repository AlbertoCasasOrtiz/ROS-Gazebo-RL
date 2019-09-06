//
// Created by alberto on 6/09/19.
//

#include "State.h"

bool State::operator==(State const &s) {
    return x == s.x && y == s.y;
}

bool State::operator!=(State const &s) {
    return x != s.x || y != s.y;
}