#include "supervisor_cross.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor_cross");
    
    supervisor_cross supervis;
    
    supervis.init();
    supervis.run();

    return 0;
}
