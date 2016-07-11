#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "math.h"
#include <sstream>

struct Force2D
{
    double fx;
    double fy;
};

struct Robot
{
    std::string robot_name;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose2D curr_pose;
    geometry_msgs::Pose2D ref;
    double err_ang;
    double err_lin_x;
    double err_lin_y;
    std::string state;
    Force2D fris;
};




class supervisor
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    
//     std::vector<ros::Subscriber> goal_subs;
    std::vector<ros::Publisher> controller_pubs;
    std::vector<Robot> robots;
   
    tf::TransformListener listener;
    
    int n;
//     double kp1 = 1;
//     double kp2 = 0.1;
//     
//     double wall1 = 0.8382;
//     double wall2 = 6.477;



private:
    void SetGoals(const geometry_msgs::Pose2D::ConstPtr& msg);
    void ReadPoses();
    void AssignGoal();
//     double AngularErr(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference);
    double LinearErrX(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference);
    double LinearErrY(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference);
//     Force2D WallRepulsion(geometry_msgs::Pose2D current);
    
    
public:
    supervisor();
    ~supervisor();
    void init();
    void run();
};

#endif //SUPERVISOR_H