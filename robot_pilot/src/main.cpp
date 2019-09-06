
#include "Pilot.h"

int main(int argc, char **argv) {
    Pilot* pilot = new Pilot(argc, argv);

    delete pilot;
    return 0;
}