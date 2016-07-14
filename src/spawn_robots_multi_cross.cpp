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

	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 3;
	    msg.initialPose.y = 14+43*i;
	    msg.initialPose.theta = 0 + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 30;
	    msg.initialPose.y = 14+43*i;
	    msg.initialPose.theta = 0 + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 70;
	    msg.initialPose.y = 14+43*i;
	    msg.initialPose.theta = 0 + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 25+38.5*i;
	    msg.initialPose.y = 3;
	    msg.initialPose.theta = M_PI/2 + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 25+38.5*i;
	    msg.initialPose.y = 30;
	    msg.initialPose.theta = M_PI/2 + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 25+38.5*i;
	    msg.initialPose.y = 70;
	    msg.initialPose.theta = M_PI/2 + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 117;
	    msg.initialPose.y = 20+43*i;
	    msg.initialPose.theta = M_PI + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 90;
	    msg.initialPose.y = 20+43*i;
	    msg.initialPose.theta = M_PI + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 50;
	    msg.initialPose.y = 20+43*i;
	    msg.initialPose.theta = M_PI + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 20+38.5*i;
	    msg.initialPose.y = 117;
	    msg.initialPose.theta = -M_PI/2 + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 20+38.5*i;
	    msg.initialPose.y = 90;
	    msg.initialPose.theta = -M_PI/2 + 0.3; // random_variable;
	
	
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
	}
	
	for(int i = 0; i < 3; i++)
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
	    msg.initialPose.x = 20+38.5*i;
	    msg.initialPose.y = 50;
	    msg.initialPose.theta = -M_PI/2 + 0.3; // random_variable;
	
	
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
	}

	ros::spinOnce();
    }
    return 0;
}