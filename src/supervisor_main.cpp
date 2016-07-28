#include "supervisor.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor");
    
    supervisor supervis;

    supervis.init();
    supervis.run();

    return 0;
}