#include "Wonderer.h"

int main(int argc, char **argv) {
    // Initiate new ROS node named "wonderer"
    ros::init(argc, argv, "wonderer");

    float proximity = 0.0;
    if (argc > 1) {
        proximity = atof(argv[1]);
    }

    // Create new wonderer object
    Wonderer wonderer(proximity);

    // Start the movement
    wonderer.start();

    return 0;
};

