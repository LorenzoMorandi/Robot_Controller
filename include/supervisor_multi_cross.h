#ifndef SUPERVISOR__MULTI_CROSS_H
#define SUPERVISOR_MULTI_CROSS_H

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <stdr_robot/handle_robot.h>
#include <stdr_parser/stdr_parser.h>
#include "math.h"
#include <sstream>
#include <iostream>
#include <random>
#include <vector>
#include <algorithm>
#include <lemon/lgf_reader.h>
#include <lemon/graph_to_eps.h>
#include <lemon/math.h>
#include <lemon/dijkstra.h>
#include <lemon/concepts/digraph.h>
#include <lemon/smart_graph.h>
#include <lemon/list_graph.h>
#include <lemon/path.h>
#include <lemon/bin_heap.h>
#include <stdr_robot/handle_robot.h>
#include <stdr_parser/stdr_parser.h>
#include <ros/package.h>
#include <cstdlib>
#include <ctime>

using namespace lemon;
using namespace std;

enum class state_machine_STATE {ROTATE_ONLY, MOVE_AND_ROTATE, MOVE_SLOW, STOP};
enum state_transition {road_free, move_rot, near_car, stop_now, rot_only};

struct Robot
{
    std::string robot_name;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose2D curr_pose;
    std::vector<geometry_msgs::Pose2D> ref;
    double err_ang;
    double err_lin;
    std::string transition;
    state_machine_STATE state;
    int id;
};

class supervisor_multi_cross
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    
    std::vector<ros::Publisher> controller_pubs;
    std::vector<Robot> robots;
   
    tf::TransformListener listener;
    
    int n;
    std::vector<int> v = {0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 83, 85, 87, 90, 91, 93, 95,98, 100, 102, 103}; //MULTI CROSS
    
private:
    void ReadPoses();
    int random_generator();
    double LinearErrX(geometry_msgs::Pose2D current, std::vector<geometry_msgs::Pose2D> reference);
    double LinearErrY(geometry_msgs::Pose2D current,  std::vector<geometry_msgs::Pose2D> reference);
    bool evolve_state_machines(int i);
    
public:
    supervisor_multi_cross();
    ~supervisor_multi_cross();
    void init();
    void run();
};

#endif //SUPERVISOR_H