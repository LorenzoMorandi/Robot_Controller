#include "supervisor.h"



void printRobot(Robot r)
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




supervisor::supervisor():pnh("~")
{
    n=0;
}

supervisor::~supervisor()
{
    for(int i=0; i<n; i++)
    {
	robots[i].twist.linear.x = 0.0;
	robots[i].twist.angular.z = 0.0;
    }   
    
    for(int i=0; i<n; i++)
	controller_pubs[i].publish(robots[i].twist);
    
    ros::spin();
}

void supervisor::ReadPoses()
{
    for(int i=0; i<n; i++)
    {
	tf::StampedTransform transform;
	try{
	    listener.waitForTransform("/world", robots[i].robot_name, ros::Time(0), ros::Duration(10.0) );
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

void supervisor::AssignGoal()
{
    
    for(int i=0; i<n; i++)
    {
	if(i==0 || i==1)
	{
	    robots[i].ref.x=37;
	    robots[i].ref.y=17;
	    robots[i].ref.theta=0;
	}
	if(i==2 || i==3)
	{
	    robots[i].ref.x=23;
	    robots[i].ref.y=37;
	    robots[i].ref.theta=0;
	}
	if(i==4 || i==5)
	{
	    robots[i].ref.x=3;
	    robots[i].ref.y=23;
	    robots[i].ref.theta=0;
	}
	if(i==6 || i==7)
	{
	    robots[i].ref.x=17;
	    robots[i].ref.y=3;
	    robots[i].ref.theta=0;
	}
    }
}

double supervisor::LinearErrY(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference)
{
    return  reference.y-current.y;
}

double supervisor::LinearErrX(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference)
{
    return  reference.x-current.x;
}

bool supervisor::evolve_state_machines(int i)
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
    std::string name="robot";
    
    for (int i=0; i<n; i++)
    {
	Robot tmp;
	tmp.robot_name=name+std::to_string(i);
	tmp.err_ang=0.0;
	tmp.err_lin=0.0;
	tmp.state=state_machine_STATE::MOVE_AND_ROTATE;
	tmp.transition="stop_now";
	robots.push_back(tmp);
	
	ros::Publisher tmp_pub;
	tmp_pub = nh.advertise<geometry_msgs::Twist>("/"+tmp.robot_name+"/cmd_vel", 1); 
	controller_pubs.push_back(tmp_pub);
    }
    
    ReadPoses();
    AssignGoal();
}

void supervisor::run()
{
    ros::Rate loop_rate(10);
    ROS_INFO_STREAM("START SUPERVISOR");

    while (ros::ok())
    {
	ReadPoses();

	for(int i=0; i<n; i++)
	{
	    double fx=LinearErrX(robots[i].curr_pose, robots[i].ref);
	    double fy=LinearErrY(robots[i].curr_pose, robots[i].ref);
	    
	    robots[i].err_ang = atan2(fy,fx)-robots[i].curr_pose.theta;
	    robots[i].err_lin = sqrt(pow(fx,2)+pow(fy,2));
	    	
	    robots[i].transition="road_free";	
	    for(int j=0; j<n; j++)
	    {
		if(i!=j)
		{
		    double gamma=atan2(robots[j].curr_pose.y-robots[i].curr_pose.y,robots[j].curr_pose.x-robots[i].curr_pose.x);
		    double theta=robots[i].curr_pose.theta;
		    double alpha=M_PI/2;
		    double dist=sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2));
		    double angle=fabs(fmod(theta-gamma, 2*M_PI));
		    
		    if(fabs(sin(robots[i].err_ang)) > 0.02)
			robots[i].transition="rot_only";
		    else if(robots[i].state==state_machine_STATE::ROTATE_ONLY)
			robots[i].transition="move_rot";
// 		    else if(dist<1.1)
// 		    {
// 			robots[i].transition="stop_now";
// 			break;
// 		    }
		    else
		    {			
			if(angle < alpha) 
			{
			    if(dist>=3 && dist<10)
				robots[i].transition="near_car";
			    if(dist<3)
				robots[i].transition="stop_now";		    
			}		    		    
		    }
		}
	    }
	}
	
	
	for(int i=0; i<n; i++)	
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
		robots[i].twist.angular.z = 2*sin(robots[i].err_ang);
		robots[i].twist.linear.x = 0.0;	
	    }
	    if(robots[i].state == state_machine_STATE::MOVE_AND_ROTATE)
	    {
		robots[i].twist.angular.z = 2*sin(robots[i].err_ang);
		robots[i].twist.linear.x = 0.1*robots[i].err_lin;	
	    }
	    if(robots[i].state == state_machine_STATE::MOVE_SLOW)
	    { 
		robots[i].twist.angular.z = 2*sin(robots[i].err_ang);
		robots[i].twist.linear.x = 0.05*robots[i].err_lin;
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
