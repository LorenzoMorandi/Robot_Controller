#include "pisa_prova.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pisa_prova");
    
    pisa_prova supervis;
    
    supervis.init();
    supervis.run();

    return 0;
}
