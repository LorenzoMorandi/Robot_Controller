#include "supervisor.h"

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
// 	    ROS_WARN_STREAM(robots[i].curr_pose);
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
//     for(int i=0; i<n; i++)
//     {
// 	if(i<4)
// 	{
// 	    robots[i].ref.x=37;
// 	    robots[i].ref.y=3;
// 	    robots[i].ref.theta=0;
// 	}
// 	else
// 	{
// 	    robots[i].ref.x=3;
// 	    robots[i].ref.y=9;
// 	    robots[i].ref.theta=0;
// 	}
//     }
    
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

// double supervisor::AngularErr(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference)
// {
//     double err_x = reference.x - current.x;
//     double err_y = reference.y - current.y;
//     double ref_theta = atan2(err_y, err_x);
// 
//     return  ref_theta - current.theta;
// }

double supervisor::LinearErrY(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference)
{
    return  reference.y-current.y;
}

double supervisor::LinearErrX(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference)
{
    return  reference.x-current.x;
}

// Force2D supervisor::WallRepulsion(geometry_msgs::Pose2D current)
// {
//     double kfr=1;
//     Force2D fr;
//     fr.fx=0;
//     fr.fy=kfr/(current.y-wall2) + kfr/(current.y-wall1);
// }


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
	tmp.err_lin_x=0.0;
	tmp.err_lin_y=0.0;
	tmp.state="MOVE";
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
    ros::Rate loop_rate(30);
    ROS_INFO_STREAM("START SUPERVISOR");

    while (ros::ok())
    {
	ReadPoses();

	for(int i=0; i<n; i++)	//twist
	{

	    Force2D fa;  
	    fa.fx=LinearErrX(robots[i].curr_pose, robots[i].ref);
	    fa.fy=LinearErrY(robots[i].curr_pose, robots[i].ref);
	    robots[i].fris.fx = fa.fx;
	    robots[i].fris.fy = fa.fy;
	    
	    double err_ang = atan2(robots[i].fris.fy,robots[i].fris.fx)-robots[i].curr_pose.theta;
	    double err_lin = sqrt(pow(robots[i].fris.fx,2)+pow(robots[i].fris.fy,2));
	    
	    if(fabs(sin(err_ang)) < 0.08)
		robots[i].twist.linear.x = 0.25*err_lin;	//MOVE
	    else
		robots[i].twist.linear.x = 0;	//ROTATE ONLY

	    robots[i].twist.angular.z = 2*sin(err_ang);	//ROTATE

	    for(int j=0; j<n; j++)
	    {
		double gamma=atan2(robots[j].curr_pose.y-robots[i].curr_pose.y,robots[j].curr_pose.x-robots[i].curr_pose.x);
		double theta=robots[i].curr_pose.theta;
		double alpha=M_PI/4;

		if(i!=j && fabs(fmod(theta-gamma, 2*M_PI))<alpha) //FIELD OF VIEW
		{
		    //FIRST EUCLIDEAN THRESHOLD
		    if(sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 10)
		    {
			if(robots[j].state=="MOVE")
			{
			    robots[i].twist.linear.x = 0.5*robots[i].twist.linear.x;
			    robots[i].state="MOVE_SLOW";
			}			
		    }
		    
		    if(sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 4 )
		    {
			    robots[i].twist.linear.x = 0.0;
			    robots[i].state="STOP";
		    }
// 		    //SECOND EUCLIDEAN THRESHOLD
// 		    if(sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 7)
// 		    {
// 			//ROBOT-GOAL DISTANCE COMPARE
// 			if(fabs(sqrt(pow(robots[i].fris.fx,2)+pow(robots[i].fris.fy,2))) > fabs(sqrt(pow(robots[j].fris.fx,2)+pow(robots[j].fris.fy,2))))
// 			{
// 			    robots[i].twist.linear.x = 0.2*robots[i].twist.linear.x;
// 			    robots[i].twist.angular.z = 0.2*robots[i].twist.angular.z;
// 			}
// 			else
// 			{
// 			    robots[i].twist.linear.x = 2*robots[i].twist.linear.x;
// 			    robots[i].twist.angular.z = 2*robots[i].twist.angular.z;
// 			}
// 		    }
// 		    //THIRD EUCLIDEAN THRESHOLD
// 		    if(sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 5)
// 		    {
// 			//ROBOT-GOAL DISTANCE COMPARE
// 			if(fabs(sqrt(pow(robots[i].fris.fx,2)+pow(robots[i].fris.fy,2))) > fabs(sqrt(pow(robots[j].fris.fx,2)+pow(robots[j].fris.fy,2))))
// 			{
// 			    robots[i].twist.linear.x = 0;
// // 			    robots[i].twist.angular.z = 0;
// 			}
// 			else
// 			{
// 			    robots[i].twist.linear.x = 2*robots[i].twist.linear.x;
// // 			    robots[i].twist.angular.z = 2*robots[i].twist.angular;
// 			}
// 		    }
		}
		else
		{
		    robots[i].state="MOVE";
		}
		    
		if(fabs(fmod(theta-gamma, 2*M_PI))>alpha/2)
		{
		    robots[i].state="MOVE";
		    robots[j].state="STOP";
		}
	    }

	}
	
	for(int i=0; i<n; i++)
	{
	    controller_pubs[i].publish(robots[i].twist);
	}   

	ros::spinOnce();
	loop_rate.sleep();
    }
}


	    
// 		if(robots[j].curr_pose.y < 6 && robots[i].curr_pose.y < 6)
// 		{
// 		    if(robots[j].curr_pose.x-robots[i].curr_pose.x > 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 3)
// 		    {
// 			robots[i].twist.linear.x = 0.5*robots[i].twist.linear.x;
// 			robots[i].twist.angular.z = 0.5*robots[i].twist.angular.z;
// 		    }
// 		    
// 		    if(robots[j].curr_pose.x-robots[i].curr_pose.x > 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 2)
// 		    {
// 			robots[i].twist.linear.x = 0.2*robots[i].twist.linear.x;
// 			robots[i].twist.angular.z = 0.2*robots[i].twist.angular.z;
// 		    }
// 		    
// 		    if(robots[j].curr_pose.x-robots[i].curr_pose.x > 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 1.8)
// 		    {
// 			robots[i].twist.linear.x = 0.0*robots[i].twist.linear.x;
// 			robots[i].twist.angular.z = 0.0*robots[i].twist.angular.z;
// 		    }
// 		}
// 		else
// 		{
// 		    if(robots[j].curr_pose.x-robots[i].curr_pose.x < 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 3)
// 		    {
// 			robots[i].twist.linear.x = 0.5*robots[i].twist.linear.x;
// 			robots[i].twist.angular.z = 0.5*robots[i].twist.angular.z;
// 		    }
// 		    
// 		    if(robots[j].curr_pose.x-robots[i].curr_pose.x < 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 2)
// 		    {
// 			robots[i].twist.linear.x = 0.2*robots[i].twist.linear.x;
// 			robots[i].twist.angular.z = 0.2*robots[i].twist.angular.z;
// 		    }
// 		    
// 		    if(robots[j].curr_pose.x-robots[i].curr_pose.x < 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 1.8)
// 		    {
// 			robots[i].twist.linear.x = 0.0*robots[i].twist.linear.x;
// 			robots[i].twist.angular.z = 0.0*robots[i].twist.angular.z;
// 		    }
// 		}
// 	    }