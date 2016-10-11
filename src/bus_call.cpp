#include "pisa_prova.h"
#include "robot_controller/call.h"


int random_generator(std::vector<int> v)
{
    std::random_shuffle (v.begin(), v.end());
    int random_variable= v[0];
    return random_variable;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "bus_call");
    
    ros::NodeHandle n;
    ros::Publisher bus_call_pub;
    
    std::vector<int> node;

    int random;
       
    bus_call_pub = n.advertise<robot_controller::call>("call", 1); 

    for (int i = 0; i < 702; i++)
    {
	node.push_back(i);
    }
    
    ros::Rate loop_rate(0.0167); //BUS CALL OGNI 60 SECONDI
           
    while (ros::ok())
    {	
	random = random_generator(node);
	
	robot_controller::call msg;
	msg.node  = random;
	
	ROS_WARN_STREAM("BUS CALL AT NODE: "<< random);
	
	bus_call_pub.publish(msg);
	
	ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
}