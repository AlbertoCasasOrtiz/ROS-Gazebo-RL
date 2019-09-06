
#include "QLearning.h"

int main(int argc, char **argv) {
    QLearning *ql = new QLearning(argc, argv);
    ql->execute();
    delete ql;
    return 0;
}