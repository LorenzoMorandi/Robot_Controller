#include "supervisor.h"

void printRobot(Robot r) //Print the robot state and transition
{
    switch(r.state)
    {
	case state_machine_STATE::ROTATE_ONLY: ROS_INFO_STREAM("ROTATE_ONLY"); break;
	case state_machine_STATE::MOVE_AND_ROTATE: ROS_INFO_STREAM("MOVE_AND_ROTATE"); break;
	case state_machine_STATE::MOVE_SLOW: ROS_INFO_STREAM("MOVE_SLOW"); break;
	case state_machine_STATE::STOP: ROS_INFO_STREAM("STOP"); break;
	default: ROS_INFO_STREAM("FAIL"); break;
    }
    ROS_INFO_STREAM(r.transition);
}

state_transition getMax(std::vector<state_transition> v) //Assign value for every transition and take the max value
{
    int v_max = 0;
    for(int i = 0; i < v.size(); i++) // Set a numeric value for every transition
    {
	int tmp_max;						//Low Priority
	if(v.at(i) == state_transition::road_free) tmp_max = 0; //	|
	if(v.at(i) == state_transition::move_rot) tmp_max = 1;	//	|
	if(v.at(i) == state_transition::near_car) tmp_max = 2;  //	|
	if(v.at(i) == state_transition::stop_now) tmp_max = 3;	//	|
	if(v.at(i) == state_transition::rot_only) tmp_max = 4;	//	|
	if(tmp_max > v_max) v_max = tmp_max;			//High priority
    }
    if(v_max == 0) return state_transition::road_free;
    if(v_max == 1) return state_transition::move_rot;
    if(v_max == 2) return state_transition::near_car;
    if(v_max == 3) return state_transition::stop_now;
    if(v_max == 4) return state_transition::rot_only;
}

supervisor::supervisor():pnh("~") //Constructor
{
    n=0;
}

supervisor::~supervisor() //Desctructor
{
    
}

void supervisor::ReadPoses() //Read from tf the robots position
{
    for(int i = 0; i < n; i++)
    {
	tf::StampedTransform transform;
	try{
	    listener.waitForTransform("/world", robots[i].robot_name, ros::Time(0), ros::Duration(1.0) );
	    listener.lookupTransform("/world", robots[i].robot_name, ros::Time(0), transform);
	    robots[i].curr_pose.x = transform.getOrigin().getX();
	    robots[i].curr_pose.y = transform.getOrigin().getY();
	    robots[i].curr_pose.theta = tf::getYaw(transform.getRotation());
	}
	catch (tf::TransformException ex){
	    ROS_ERROR("%s",ex.what());
	    ros::Duration(1.0).sleep();
	}
    }
}

void supervisor::AssignGoal() //Manually assignement of goal
{
    geometry_msgs::Pose2D tmp;
    
    //GOAL FOR MULTI CROSS SCENARIO
    
//     for(int i = 0; i < n; i++)
//     {
// 	if(i>=0 && i<=8)
// 	{
// 	    tmp.y = robots[i].curr_pose.y;
// 	    tmp.x = 118;	    
// 	    robots[i].ref.push_back(tmp);
// 	}
// 	
// 	if(i>=9 && i<=17)
// 	{
// 	    tmp.y = 118;
// 	    tmp.x = robots[i].curr_pose.x;	    
// 	    robots[i].ref.push_back(tmp);
// 	}
// 	
// 	if(i>=18 && i<=26)
// 	{
// 	    tmp.y = robots[i].curr_pose.y;
// 	    tmp.x = 2;	    
// 	    robots[i].ref.push_back(tmp);
// 	}
// 	
// 	if(i>=27 && i<=36)
// 	{
// 	    tmp.y = 2;
// 	    tmp.x = robots[i].curr_pose.x;	    
// 	    robots[i].ref.push_back(tmp);
// 	}
//     }

    //GOAL FOR SINGLE CROSS SCENARIO
    
    //robot 0 goal sequences
    tmp.y = robots[0].curr_pose.y;
    tmp.x = 14;	    
    robots[0].ref.push_back(tmp);
    tmp.x = 23;	 
    tmp.y = 25;
    robots[0].ref.push_back(tmp);
    tmp.x = 23;	 
    tmp.y = 35;
    robots[0].ref.push_back(tmp);
    
    //robot 1 goal sequences
    tmp.x = 17;	 
    tmp.y = 15; 
    robots[1].ref.push_back(tmp);
    tmp.x = 17;	 
    tmp.y = 5; 
    robots[1].ref.push_back(tmp);
    
    //robot 2 goal sequences
    tmp.x = robots[2].curr_pose.x;
    tmp.y = 10;	    
    robots[2].ref.push_back(tmp);
    tmp.y = 20;	    
    robots[2].ref.push_back(tmp);
    tmp.y = 38;	    
    robots[2].ref.push_back(tmp);
    
    //robot 3 goal sequences
    tmp.x = robots[3].curr_pose.x;
    tmp.y = 20;	    
    robots[3].ref.push_back(tmp);
    tmp.y = 38;	    
    robots[3].ref.push_back(tmp);
    
    //robot 4 goal sequences
    tmp.y = robots[4].curr_pose.y;
    tmp.x = 30;	    
    robots[4].ref.push_back(tmp);
    tmp.x = 20;	    
    robots[4].ref.push_back(tmp);
    tmp.x = 5;	    
    robots[4].ref.push_back(tmp);
    
    //robot 5 goal sequences
    tmp.y = robots[5].curr_pose.y;
    tmp.x = 20;	    
    robots[5].ref.push_back(tmp);
    tmp.x = 5;	    
    robots[5].ref.push_back(tmp);
    
    //robot 6 goal sequences
    tmp.x = robots[6].curr_pose.x;
    tmp.y = 30;	    
    robots[6].ref.push_back(tmp);
    tmp.y = 20;	    
    robots[6].ref.push_back(tmp);
    tmp.y = 5;	    
    robots[6].ref.push_back(tmp);
    
    //robot 7 goal sequences
    tmp.x = robots[7].curr_pose.x;
    tmp.y = 20;	    
    robots[7].ref.push_back(tmp);
    tmp.y = 5;	    
    robots[7].ref.push_back(tmp);
}

double supervisor::LinearErrY(geometry_msgs::Pose2D current, std::vector<geometry_msgs::Pose2D> reference)
{
    geometry_msgs::Pose2D reff = reference[0];
    return  reff.y - current.y;
}

double supervisor::LinearErrX(geometry_msgs::Pose2D current, std::vector<geometry_msgs::Pose2D> reference)
{
    geometry_msgs::Pose2D reff = reference[0];
    return  reff.x - current.x;
}

bool supervisor::evolve_state_machines(int i)	//State Machine evolution
{
    if(robots[i].transition == "stop_now" && robots[i].state == state_machine_STATE::MOVE_AND_ROTATE)
    {
	robots[i].state = state_machine_STATE::STOP;
	return true;
    }
    if(robots[i].transition == "move_rot" && robots[i].state == state_machine_STATE::ROTATE_ONLY)
    {
	robots[i].state = state_machine_STATE::MOVE_AND_ROTATE;
	return true;
    }
    if(robots[i].transition == "rot_only" && robots[i].state == state_machine_STATE::MOVE_AND_ROTATE)
    {
	robots[i].state = state_machine_STATE::ROTATE_ONLY;
	return true;
    }
    if(robots[i].transition == "near_car" && robots[i].state == state_machine_STATE::MOVE_AND_ROTATE)
    {
	robots[i].state = state_machine_STATE::MOVE_SLOW;
	return true;
    }
    if(robots[i].transition == "road_free" && robots[i].state == state_machine_STATE::MOVE_SLOW)
    {
	robots[i].state = state_machine_STATE::MOVE_AND_ROTATE;
	return true;
    }
    if(robots[i].transition == "stop_now" && robots[i].state == state_machine_STATE::MOVE_SLOW)
    {
	robots[i].state = state_machine_STATE::STOP;
	return true;
    }
    if(robots[i].transition == "road_free" && robots[i].state == state_machine_STATE::STOP)
    {
	robots[i].state = state_machine_STATE::MOVE_SLOW;
	return true;
    }
    if(robots[i].transition == "rot_only" && robots[i].state == state_machine_STATE::MOVE_SLOW)
    {
	robots[i].state = state_machine_STATE::ROTATE_ONLY;
	return true;
    }
    if(robots[i].transition == "rot_only" && robots[i].state == state_machine_STATE::STOP)
    {
	robots[i].state = state_machine_STATE::ROTATE_ONLY;
	return true;
    }
    return false;
}


void supervisor::init()
{
    pnh.param<int>("robot_number", n, 8);
    ROS_INFO_STREAM("Robot Number: " << n);
    std::string name = "robot";
    
    for (int i = 0; i < n; i++)
    {
	Robot tmp;
	tmp.robot_name = name + std::to_string(i);
	tmp.err_ang = 0.0;
	tmp.err_lin = 0.0;
	tmp.state = state_machine_STATE::MOVE_AND_ROTATE;
	tmp.transition = "stop_now";
	tmp.id = i;
	robots.push_back(tmp);
	
	ros::Publisher tmp_pub;
	tmp_pub = nh.advertise<geometry_msgs::Twist>("/" + tmp.robot_name + "/cmd_vel", 100); 
	controller_pubs.push_back(tmp_pub);
    }
    
    ReadPoses();
    AssignGoal();
}

void supervisor::run()
{
    ros::Rate loop_rate(30);
    ROS_INFO_STREAM("START SUPERVISOR");

    
    while (ros::ok())
    {
	ReadPoses();
	
	std::vector<state_transition> tmp(n, state_transition::road_free);
	std::vector<std::vector<state_transition>> matrix(n, tmp);	//matrix nxn containing state transition info (i,j) e (j,i) 
	
	for(int i = 0; i < n; i++)
	{
	    double fx = LinearErrX(robots[i].curr_pose, robots[i].ref);
	    double fy = LinearErrY(robots[i].curr_pose, robots[i].ref);
	    
	    //Compute linear and angular error for robot i
	    robots[i].err_ang = atan2(fy,fx) - robots[i].curr_pose.theta + 0.01;
	    robots[i].err_lin = sqrt(pow(fx,2) + pow(fy,2));
	    	    	    
	    if(robots[i].err_lin < 1 && robots[i].ref.size()>1)
		robots[i].ref.erase(robots[i].ref.begin()+0); //SWITCH GOAL
	    
	    
	    //Robot i look at all the other robots j
	    for(int j = 0; j < n; j++)
	    {
		if(i != j)
		{
		    double gamma = atan2(robots[j].curr_pose.y - robots[i].curr_pose.y, robots[j].curr_pose.x - robots[i].curr_pose.x); //angle between horizontal and the rect connect i and j
		    double theta = robots[i].curr_pose.theta; //current orientation of i
		    double alpha= M_PI/5; //half vision angle    
		    double angle = fabs(fmod(theta - gamma, 2*M_PI)); 
		    
		    double dist = sqrt(pow(robots[i].curr_pose.x - robots[j].curr_pose.x,2) + pow(robots[i].curr_pose.y - robots[j].curr_pose.y,2)); //distance between i and j

// 		    if (i < j)
// 			double alpha = M_PI; //half vision angle
		    
		    if(fabs(sin(robots[i].err_ang)) > 0.05)
		    {
			matrix.at(i).at(j) = state_transition::rot_only;
		    }
		    else if(robots[i].state == state_machine_STATE::ROTATE_ONLY)
		    {
			matrix.at(i).at(j) = state_transition::move_rot;
		    }
		    else
		    {			
			if(angle < alpha) //j is in the vision range of i
			{
			    if(dist >= 5 && dist < 8)
			    {
				matrix.at(i).at(j) = state_transition::near_car;
			    }
			    
			    if(dist < 5)
			    {
				matrix.at(i).at(j) = state_transition::stop_now;
				
				if(robots[i].id > robots[j].id && dist > 3) 
				    matrix.at(i).at(j) = state_transition::road_free;
				if(robots[i].id > robots[j].id && dist < 3)
				    matrix.at(i).at(j) = state_transition::stop_now;
			    }

			}
			
			/*if(robots[i].curr_pose.x <= 14.5 && robots[i].curr_pose.x >= 13.5 &&  robots[i].curr_pose.y <= 17.5 && robots[i].curr_pose.y >= 16.5 || 
			    robots[i].curr_pose.x <= 23.5 && robots[i].curr_pose.x >= 22.5 && robots[i].curr_pose.y <= 14.5 && robots[i].curr_pose.y >= 13.5||
			    robots[i].curr_pose.x <= 26.5 && robots[i].curr_pose.x >= 25.5 && robots[i].curr_pose.y <= 23.5 && robots[i].curr_pose.y >= 22.5 ||
			    robots[i].curr_pose.x <= 17.5 && robots[i].curr_pose.x >= 16.5 && robots[i].curr_pose.y <= 26.5 && robots[i].curr_pose.y >= 25.5 )
			{  
			    alpha*=2; //half vision angle    
			    
			    if(angle < alpha && dist < 13)
				if(i > j)
				    matrix.at(i).at(j) = state_transition::road_free;
				else
				    matrix.at(i).at(j) = state_transition::stop_now;
			}	*/	    		    
		    }
		}
	    }
	    
	    if(robots[i].err_lin < 0.5) //DELETE ROBOT
	    {
		stdr_robot::HandleRobot handler;
		std::string name("robot" + std::to_string(robots[i].id));
		try
		{    
		    if (handler.deleteRobot(name))
		    {
			ROS_INFO("Robot %s deleted successfully", name.c_str());
		    }
		    else 
		    {
			ROS_ERROR("Could not delete robot %s", name.c_str());
		    }
		}
		catch (stdr_robot::ConnectionException& ex) 
		{
		    ROS_ERROR("%s", ex.what());
		}
		robots.erase(robots.begin() + i);
		n--;
	    }
	}
		
	for(int i = 0; i < n; i++)
	{
	    switch(getMax(matrix.at(i)))
	    {
		case state_transition::road_free: robots[i].transition = "road_free"; break;
		case state_transition::rot_only: robots[i].transition = "rot_only";  break;
		case state_transition::near_car: robots[i].transition = "near_car"; break;
		case state_transition::stop_now: robots[i].transition = "stop_now"; break;
		case state_transition::move_rot: robots[i].transition = "move_rot"; break;
		default: abort();
	    }
	}
	
	for(int i = 0; i < n; i++)	
	{
	    if(evolve_state_machines(i))
	    {
		switch(robots[i].state)
		{
		    case state_machine_STATE::ROTATE_ONLY: ROS_WARN_STREAM("Robot " << std::to_string(i) << ": ROTATE_ONLY"); break;
		    case state_machine_STATE::MOVE_AND_ROTATE: ROS_WARN_STREAM("Robot " << std::to_string(i) << ": MOVE_AND_ROTATE"); break;
		    case state_machine_STATE::MOVE_SLOW: ROS_WARN_STREAM("Robot " << std::to_string(i) << ": MOVE_SLOW"); break;
		    case state_machine_STATE::STOP: ROS_WARN_STREAM("Robot " << std::to_string(i) << ": STOP"); break;
		    default: ROS_WARN_STREAM("FAIL"); break;
		}
	    }
	        
	    if(robots[i].state == state_machine_STATE::ROTATE_ONLY)
	    {
		robots[i].twist.angular.z = 3*sin(robots[i].err_ang);
		robots[i].twist.linear.x = 0.0;	
	    }
	    if(robots[i].state == state_machine_STATE::MOVE_AND_ROTATE)
	    {
		robots[i].twist.angular.z = 3*sin(robots[i].err_ang);
		robots[i].twist.linear.x = 0.5*robots[i].err_lin;	
	    }
	    if(robots[i].state == state_machine_STATE::MOVE_SLOW)
	    { 
		robots[i].twist.angular.z = 3*sin(robots[i].err_ang);
		robots[i].twist.linear.x = 0.2*robots[i].err_lin;
	    }
	    if(robots[i].state == state_machine_STATE::STOP)
	    {
		robots[i].twist.angular.z = 0.0;
		robots[i].twist.linear.x = 0.0;	
	    }
	    controller_pubs[i].publish(robots[i].twist);
	}

	ros::spinOnce();
	loop_rate.sleep();
    }
}
