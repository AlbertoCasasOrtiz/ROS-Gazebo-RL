//
// Created by alberto on 6/09/19.
//

#ifndef SRC_RANDOMGENERATOR_H
#define SRC_RANDOMGENERATOR_H

#include <time.h>
#include <random>

class RandomGenerator {
private:
    /// Seeder of the random number generator.
    static std::mt19937 mt;
public:
    /// Get float random value in a given interval.
    /// \param start Start of interval.
    /// \param end End of interval.
    /// \return Float random value.
    float static getFloat(float start, float end);

    /// Get integer random value in a given interval.
    /// \param start Start of interval.
    /// \param end End of interval.
    /// \return Integer random value.
    int static getInt(int start, int end);
};


#endif //SRC_RANDOMGENERATOR_H
