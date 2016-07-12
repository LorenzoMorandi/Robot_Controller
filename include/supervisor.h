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


enum class state_machine_STATE {ROTATE_ONLY, MOVE_AND_ROTATE, MOVE_SLOW, STOP};

struct Robot
{
    std::string robot_name;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose2D curr_pose;
    geometry_msgs::Pose2D ref;
    double err_ang;
    double err_lin;
    std::string transition;
    state_machine_STATE state;
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

private:
    void SetGoals(const geometry_msgs::Pose2D::ConstPtr& msg);
    void ReadPoses();
    void AssignGoal();
    double LinearErrX(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference);
    double LinearErrY(geometry_msgs::Pose2D current, geometry_msgs::Pose2D reference);
    bool evolve_state_machines(int i);
    
    
public:
    supervisor();
    ~supervisor();
    void init();
    void run();
};

#endif //SUPERVISOR_H