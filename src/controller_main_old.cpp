#include "controller.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    
    controller contr;

    contr.init();
    contr.run();

    return 0;
}