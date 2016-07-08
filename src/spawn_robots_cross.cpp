#include <stdr_robot/handle_robot.h>
#include <stdr_parser/stdr_parser.h>
#include <ros/package.h>
#include <cstdlib>
#include <iostream>
#include <ctime>

 
 int main(int argc, char **argv)
{
    ros::init(argc, argv, "spawn");
    	
    stdr_robot::HandleRobot handler;
    std::srand(std::time(0));
   
    if(ros::ok())
    {
	stdr_msgs::RobotMsg msg;
	std::string robot_type = ros::package::getPath("robot_controller") + "/robots/simple_robot.xml";

	for(int i = 0; i < 1; i++)
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
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x = 3*i+3;
	    msg.initialPose.y = 17;
	    msg.initialPose.theta = 0 + random_variable;
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
	    }
	    catch (stdr_robot::ConnectionException& ex) 
	    {
		ROS_ERROR("%s", ex.what());
		return -1;
	    }
	    usleep(100*1000);
	}
	
	for(int i = 0; i < 1; i++)
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
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x = 23;
	    msg.initialPose.y = 3*i+3;
	    msg.initialPose.theta = 0 + random_variable;
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
	    }
	    catch (stdr_robot::ConnectionException& ex) 
	    {
		ROS_ERROR("%s", ex.what());
		return -1;
	    }
	    usleep(100*1000);
	}
	
	for(int i = 0; i < 1; i++)
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
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x = 37-3*i;
	    msg.initialPose.y = 23;
	    msg.initialPose.theta = M_PI + random_variable;
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
	    }
	    catch (stdr_robot::ConnectionException& ex) 
	    {
		ROS_ERROR("%s", ex.what());
		return -1;
	    }
	    usleep(100*1000);
	}

	    
	for(int i = 0; i < 1; i++)
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
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x = 17;
	    msg.initialPose.y = 37-3*i;
	    msg.initialPose.theta = M_PI + random_variable;
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
	    }
	    catch (stdr_robot::ConnectionException& ex) 
	    {
		ROS_ERROR("%s", ex.what());
		return -1;
	    }
	    usleep(100*1000);
	}
	ros::spinOnce();
    }
    return 0;
}