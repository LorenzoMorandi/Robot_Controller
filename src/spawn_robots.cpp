#include <stdr_robot/handle_robot.h>
#include <stdr_parser/stdr_parser.h>
#include <ros/package.h>

 
 int main(int argc, char **argv)
{
    ros::init(argc, argv, "spawn");
    	
    stdr_robot::HandleRobot handler;
   
    if(ros::ok())
    {
	stdr_msgs::RobotMsg msg;
	std::string robot_type = ros::package::getPath("robot_controller") + "/robots/simple_robot.xml";
	for(int i = 0; i < 2; i++)
	{
	    try 
	    {
		msg = stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>(robot_type);
	    }
	    catch(stdr_parser::ParserException& ex)
	    {
		ROS_ERROR("[STDR_PARSER] %s", ex.what());
		return -1;
	    }
	    
	    msg.initialPose.x = 5*i+5;
	    msg.initialPose.y = 2;
	    msg.initialPose.theta = 0;
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
// 		return 0;
	    }
	    catch (stdr_robot::ConnectionException& ex) {
		ROS_ERROR("%s", ex.what());
// 		return -1;
	    }
	    usleep(100*1000);
	}
	
	for(int i = 0; i < 0; i++)
	{
	    try 
	    {
		msg = stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>(robot_type);
	    }
	    catch(stdr_parser::ParserException& ex)
	    {
		ROS_ERROR("[STDR_PARSER] %s", ex.what());
		return -1;
	    }
	    
	    msg.initialPose.x = 35-5*i;
	    msg.initialPose.y = 8;
	    msg.initialPose.theta = M_PI;
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
// 		return 0;
	    }
	    catch (stdr_robot::ConnectionException& ex) {
		ROS_ERROR("%s", ex.what());
// 		return -1;
	    }
	    usleep(100*1000);
	}
	ros::spinOnce();
    }
    return 0;
}