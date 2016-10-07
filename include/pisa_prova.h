#ifndef PISA_PROVA_H
#define PISA_PROVA_H

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <handle_robot.h>
#include <stdr_parser.h>
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
#include <ros/package.h>
#include <cstdlib>
#include <ctime>
#include "robot_controller/robot.h"

using namespace lemon;
using namespace std;

enum class state_machine_STATE {ROTATE_ONLY, MOVE_AND_ROTATE, MOVE_SLOW, STOP};
enum state_transition {road_free, move_rot, near_car, stop_now, rot_only};

typedef SmartDigraph Graph;   
typedef SmartDigraph::ArcMap<double> LengthMap;
typedef SmartDigraph::Node Node;
typedef SmartDigraph::Arc Edge;
typedef SmartDigraph::NodeMap<double> CoordMap;
typedef dim2::Point<double> Point;
    
struct Robot
{
    std::string robot_name;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose2D curr_pose;
    std::vector<geometry_msgs::Pose2D> ref;
    std::vector<Node> ref_node;
    std::vector<Node> goal_node;
    Node prev_ref_node;
    double err_ang;
    double err_lin;
    std::string transition;
    state_machine_STATE state;
    int robot_state;
    int prev_state;
    int id;
    int prev_value;
    bool public_robot;
    int bus_stop_counter;
};

class pisa_prova
{
public:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    
    std::vector<ros::Publisher> controller_pubs;
    std::vector<ros::Publisher> robot_pubs;
    std::vector<Robot> robots;
    std::vector<Robot> public_robots;
   
    tf::TransformListener listener;
    
    int n;
    int public_n;
    int number;
    int var=0;
    double middleweight;

    std::vector<int> node;
    std::vector<Node> random_start_node;
    std::vector<Node> random_goal_node;
    std::vector<Node> public_start_node;
    std::vector<Node> public_goal_node;

//     typedef SmartDigraph::NodeMap<int> IdMap;
    
    Graph g;
    LengthMap len;
    LengthMap init_len;
    CoordMap coord_x;
    CoordMap coord_y;
    SmartDigraph::NodeMap<Point> coords;
//     IdMap id;
    
public:
    void ReadPoses();
    state_transition getMax(std::vector<state_transition> v);
    double special_sin(double err_ang); 
    int random_generator(std::vector<int> v);
    double LinearErrX(geometry_msgs::Pose2D current, std::vector<geometry_msgs::Pose2D> reference);
    double LinearErrY(geometry_msgs::Pose2D current,  std::vector<geometry_msgs::Pose2D> reference);
    bool evolve_state_machines(int i);
    bool evolve_state_machines_public(int i);
    void loadGraph();
    void spawnRobot();
    void spawnPublicRobot();
    void initRobot();
    void computePath();
    void computePublicPath();
    
    pisa_prova();
    ~pisa_prova();
    void init();
    void run();
};

#endif //PISA_PROVA_H