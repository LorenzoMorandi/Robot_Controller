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
	controller_pubs[i].publish(robots[i].twist);
    }
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
    for(int i=0; i<n; i++)
    {
// 	robots[i].ref.x=
// 	robots[i].ref.y=
// 	robots[i].ref.theta=
    }
    robots[0].ref.x=37;
    robots[0].ref.y=2;
    robots[0].ref.theta=0;
    
    robots[1].ref.x=37;
    robots[1].ref.y=2;
    robots[1].ref.theta=0;
    
    robots[2].ref.x=37;
    robots[2].ref.y=4;
    robots[2].ref.theta=0;
    
    robots[3].ref.x=37;
    robots[3].ref.y=4;
    robots[3].ref.theta=0;
    
    robots[4].ref.x=3;
    robots[4].ref.y=8;
    robots[4].ref.theta=0;
    
    robots[5].ref.x=3;
    robots[5].ref.y=8;
    robots[5].ref.theta=0;
    
    robots[6].ref.x=3;
    robots[6].ref.y=10;
    robots[6].ref.theta=0;
    
    robots[7].ref.x=3;
    robots[7].ref.y=10;
    robots[7].ref.theta=0;
    
//     robots[8].ref.x=5;
//     robots[8].ref.y=8;
//     robots[8].ref.theta=0;
//     
//     robots[9].ref.x=5;
//     robots[9].ref.y=8;
//     robots[9].ref.theta=0;
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

	    Force2D fa;   //fwall;
	    fa.fx=LinearErrX(robots[i].curr_pose, robots[i].ref);
	    fa.fy=LinearErrY(robots[i].curr_pose, robots[i].ref);
	    robots[i].fris.fx = fa.fx;// + fwall.fx;
	    robots[i].fris.fy = fa.fy;// + fwall.fy;
	    
	    double err_ang=atan2f(robots[i].fris.fy,robots[i].fris.fx)-robots[i].curr_pose.theta;
	    
	    if(fabs(sin(err_ang)) < 0.1)
		robots[i].twist.linear.x = 0.2*sqrt(pow(robots[i].fris.fx,2)+pow(robots[i].fris.fy,2));
	    else
		robots[i].twist.linear.x = 0;

	    robots[i].twist.angular.z = 2*sin(err_ang);

	    for(int j=0; j<n; j++)
	    {
		if(robots[j].curr_pose.y < 6 && robots[i].curr_pose.y < 6)
		{
		    if(robots[j].curr_pose.x-robots[i].curr_pose.x > 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 3)
		    {
			robots[i].twist.linear.x = 0.5*robots[i].twist.linear.x;
			robots[i].twist.angular.z = 0.5*robots[i].twist.angular.z;
		    }
		    
		    if(robots[j].curr_pose.x-robots[i].curr_pose.x > 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 2)
		    {
			robots[i].twist.linear.x = 0.2*robots[i].twist.linear.x;
			robots[i].twist.angular.z = 0.2*robots[i].twist.angular.z;
		    }
		    
		    if(robots[j].curr_pose.x-robots[i].curr_pose.x > 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 1.8)
		    {
			robots[i].twist.linear.x = 0.0*robots[i].twist.linear.x;
			robots[i].twist.angular.z = 0.0*robots[i].twist.angular.z;
		    }
		}
		else
		{
		    if(robots[j].curr_pose.x-robots[i].curr_pose.x < 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 3)
		    {
			robots[i].twist.linear.x = 0.5*robots[i].twist.linear.x;
			robots[i].twist.angular.z = 0.5*robots[i].twist.angular.z;
		    }
		    
		    if(robots[j].curr_pose.x-robots[i].curr_pose.x < 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 2)
		    {
			robots[i].twist.linear.x = 0.2*robots[i].twist.linear.x;
			robots[i].twist.angular.z = 0.2*robots[i].twist.angular.z;
		    }
		    
		    if(robots[j].curr_pose.x-robots[i].curr_pose.x < 0 && sqrt(pow(robots[i].curr_pose.x-robots[j].curr_pose.x,2)+pow(robots[i].curr_pose.y-robots[j].curr_pose.y,2)) < 1.8)
		    {
			robots[i].twist.linear.x = 0.0*robots[i].twist.linear.x;
			robots[i].twist.angular.z = 0.0*robots[i].twist.angular.z;
		    }
		}
	    }
	}
	
	for(int i=0; i<n; i++)
	    controller_pubs[i].publish(robots[i].twist);

	    

	ros::spinOnce();
	loop_rate.sleep();
    }
}