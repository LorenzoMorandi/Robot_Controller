#include "supervisor_multi_cross.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor_multi_cross");
    
    supervisor_multi_cross supervis;
    
    supervis.init();
    supervis.run();

    return 0;
}
