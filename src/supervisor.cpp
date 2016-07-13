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
    for(int i=0; i<n; i++)
    {
	robots[i].twist.linear.x = 0.0;
	robots[i].twist.angular.z = 0.0;
    }   
    
    for(int i = 0; i < n; i++)
	controller_pubs[i].publish(robots[i].twist);
    
    ros::spin();
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


void supervisor::SetGoals(const geometry_msgs::Pose2D::ConstPtr& msg)
{
//TODO
}

void supervisor::AssignGoal() //Manually assignement of goal
{
    for(int i = 0; i < n; i++)
    {
	if(i == 0)
	{
	    robots[i].ref.x = 37;
	    robots[i].ref.y = 17;
	    robots[i].ref.theta = 0;
	}
	if(i == 1)
	{
	    robots[i].ref.x = 37;
	    robots[i].ref.y = 17;
	    robots[i].ref.theta = 0;
	}
	if(i == 2)
	{
	    robots[i].ref.x = 23;
	    robots[i].ref.y = 37;
	    robots[i].ref.theta = 0;
	}
	if(i == 3)
	{
	    robots[i].ref.x = 23;
	    robots[i].ref.y = 37;
	    robots[i].ref.theta = 0;
	}
	if(i == 4)
	{
	    robots[i].ref.x = 3;
	    robots[i].ref.y = 23;
	    robots[i].ref.theta = 0;
	}
	if(i == 5)
	{
	    robots[i].ref.x = 3;
	    robots[i].ref.y = 23;
	    robots[i].ref.theta = 0;
	}
	if(i == 6)
	{
	    robots[i].ref.x = 17;
	    robots[i].ref.y = 3;
	    robots[i].ref.theta = 0;
	}
	if(i == 7)
	{
	    robots[i].ref.x = 17;
	    robots[i].ref.y = 3;
	    robots[i].ref.theta = 0;
	}
    }
}

double supervisor::LinearErrY(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference)
{
    return  reference.y - current.y;
}

double supervisor::LinearErrX(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference)
{
    return  reference.x - current.x;
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
	    	
	    //Robot i look at all the other robots j
	    for(int j = 0; j < n; j++)
	    {
		if(i != j)
		{
		    double gamma = atan2(robots[j].curr_pose.y - robots[i].curr_pose.y, robots[j].curr_pose.x - robots[i].curr_pose.x); //angle between horizontal and the rect connect i and j
		    double theta = robots[i].curr_pose.theta; //current orientation of i
		    double alpha = M_PI/5; //half vision angle
		    double angle = fabs(fmod(theta - gamma, 2*M_PI)); 
		    
		    double dist = sqrt(pow(robots[i].curr_pose.x - robots[j].curr_pose.x,2) + pow(robots[i].curr_pose.y - robots[j].curr_pose.y,2)); //distance between i and j
		    double goaldist_i = sqrt(pow(robots[i].ref.x - robots[i].curr_pose.x,2) + pow(robots[i].ref.y - robots[i].curr_pose.y,2)); //distance between i and goal
		    double goaldist_j = sqrt(pow(robots[j].ref.x - robots[j].curr_pose.x,2) + pow(robots[j].ref.y - robots[j].curr_pose.y,2)); //distance between j and goal

		    if(fabs(sin(robots[i].err_ang)) > 0.1)
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
			    if(dist >= 4 && dist < 12)
			    {
				matrix.at(i).at(j) = state_transition::near_car;
			    }
			    if(dist < 4)
			    {
				matrix.at(i).at(j) = state_transition::stop_now;
				
				if(i > j) 
				    matrix.at(i).at(j) = state_transition::road_free;
			    }
			}		    		    
		    }
		}
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
		robots[i].twist.linear.x = 0.05*robots[i].err_lin;	
	    }
	    if(robots[i].state == state_machine_STATE::MOVE_SLOW)
	    { 
		robots[i].twist.angular.z = 3*sin(robots[i].err_ang);
		robots[i].twist.linear.x = 0.02*robots[i].err_lin;
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
