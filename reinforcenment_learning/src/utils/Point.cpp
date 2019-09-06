//
// Created by alberto on 6/09/19.
//

#include <string>
#include "Point.h"

Point::Point(int x, int y) {
    Point::x = x;
    Point::y = y;
}

std::string Point::toString() {
    return "(" + std::to_string(Point::x) + ", " + std::to_string(Point::y) + ")";
}
