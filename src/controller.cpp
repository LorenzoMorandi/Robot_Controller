#include "controller.h"


controller::controller()
{
    
}

controller::~controller()
{
    
}

void controller::ReadCurPos()
{
    tf::StampedTransform transform;
    try{
	listener.waitForTransform("/world", robot_id, ros::Time(0), ros::Duration(10.0) );
	listener.lookupTransform("/world", robot_id, ros::Time(0), transform);
	curr_pose.position.x = transform.getOrigin().getX();
	curr_pose.position.y = transform.getOrigin().getY();
	curr_pose.position.z = transform.getOrigin().getZ();
	curr_pose.orientation.x = transform.getRotation().getX();
	curr_pose.orientation.y = transform.getRotation().getY();
	curr_pose.orientation.z = transform.getRotation().getZ();
	curr_pose.orientation.w = transform.getRotation().getW();
// 	double yaw = tf::getYaw(transform.getRotation());
    }
    catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
    }
}

void controller::SetGoal(const geometry_msgs::Pose::ConstPtr& msg)
{
    ref=*msg;
}

double controller::AngularErr(geometry_msgs::Pose current, geometry_msgs::Pose reference)
{
    double err_x = reference.position.x - current.position.x;
    double err_y = reference.position.y - current.position.y;
    double ref_theta = atan2f(err_y, err_x);
    tf::Quaternion q(current.orientation.x, current.orientation.y, current.orientation.z, current.orientation.w);
    double yaw=tf::getYaw(q);
    return  ref_theta - yaw;
}

double controller::LinearErr(geometry_msgs::Pose current, geometry_msgs::Pose reference)
{
    return  sqrt(pow((reference.position.y-current.position.y),2)+pow((reference.position.x-current.position.x),2));
}

// void controller::callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
// //   ROS_INFO("Reconfigure Request: %f %f %f %f", /*%s %s %d*/
// //             kp1=config.kp1, 
// // 	    ki1=config.ki1,
// // 	    kp2=config.kp2, 
// // 	    ki2=config.ki2); 
// 	    kp1 = config.kp1;
// 	    kp2 = config.kp2;
// 	    ki1 = config.ki1;
// 	    ki2 = config.ki2;
// //             config.str_param.c_str(), 
// //             config.bool_param?"True":"False", 
// //             config.size);
// }

void controller::init()
{
//     f = boost::bind(&controller::callback,this, _1, _2);
//     server.setCallback(f);
    
    goal_sub = n.subscribe("goal", 1, &controller::SetGoal, this);
    controller_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    robot_id = n.getNamespace().c_str();
    ROS_WARN_STREAM(robot_id);
}

void controller::run()
{
    ros::Rate loop_rate(30);
    
    int count = 0;
    
    ReadCurPos();
    ref=curr_pose;
    
    while (ros::ok())
    {
	ReadCurPos();
	
	err_lin = LinearErr(curr_pose, ref);
	err_ang = AngularErr(curr_pose, ref);
	      
	err_lin_old += err_lin;
        err_ang_old += err_ang;
        
        if(err_lin > 0.01)
        {
	    if (sin(err_ang) > 0.005 || sin(err_ang) < -0.005)
	    {
		twist.linear.x = 0;
		twist.angular.z = kp2*sin(err_ang) + ki2*sin(err_ang_old);
	    }
	    else
	    {
		twist.linear.x = kp1*err_lin + ki1*err_lin_old;
		twist.angular.z = 0;
	    }
	}
        else 
        {
	    twist.linear.x = 0;
            twist.angular.z = 0;
	    err_lin_old=0;
	    err_ang_old=0;
        }
	
	if(count%300 == 0)
	{
	    ROS_INFO_STREAM("angular error: " << err_ang*180/M_PI << "\tlinear error: " << err_lin);
	    ROS_INFO_STREAM("Curr: (" << curr_pose.position.x << " " << curr_pose.position.y << ")");
	    ROS_WARN_STREAM("Ref: (" << ref.position.x << " " << ref.position.y << ")");
	    count = 0;
	}
	
	controller_pub.publish(twist);

	ros::spinOnce();
	loop_rate.sleep();
	++count;
    }
}
