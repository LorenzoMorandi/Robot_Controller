#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/console.h"
#include <geometry_msgs/Pose.h>
#include "math.h"
#include <sstream>
#include <tf/tf.h>
#include <tf/transform_listener.h>


// #include <dynamic_reconfigure/server.h>
// #include <controller/TutorialsConfig.h>


class controller
{
private:
    ros::NodeHandle n;
    ros::Subscriber goal_sub;
    ros::Publisher controller_pub;
    std::string robot_id;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose curr_pose;
    geometry_msgs::Pose ref;
    tf::TransformListener listener;
    
    double err_ang = 0.0;
    double err_lin = 0.0;
    double err_ang_old = 0.0;
    double err_lin_old= 0.0;

//     dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
//     dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;

    
public:
    
    double kp1= 0.5;
    double ki1= 0;
    double kp2= 2.5;
    double ki2= 0;
            
private:
    
    void SetGoal(const geometry_msgs::Pose::ConstPtr& msg);
    void ReadCurPos();
    double AngularErr(geometry_msgs::Pose current, geometry_msgs::Pose reference);
    double LinearErr(geometry_msgs::Pose current, geometry_msgs::Pose reference);
    
public:
    
    controller();
    ~controller();
    void init();
    void run();
//     void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level);
};

#endif //CONTROLLER_H