#include "supervisor_cross.h"

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

supervisor_cross::supervisor_cross():pnh("~") //Constructor
{
    n=0;
}

supervisor_cross::~supervisor_cross() //Desctructor
{
    
}

void supervisor_cross::ReadPoses() //Read from tf the robots position
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

double supervisor_cross::LinearErrY(geometry_msgs::Pose2D current, std::vector<geometry_msgs::Pose2D> reference)
{
    geometry_msgs::Pose2D reff = reference[0];
    return  reff.y - current.y;
}

double supervisor_cross::LinearErrX(geometry_msgs::Pose2D current, std::vector<geometry_msgs::Pose2D> reference)
{
    geometry_msgs::Pose2D reff = reference[0];
    return  reff.x - current.x;
}

bool supervisor_cross::evolve_state_machines(int i)	//State Machine evolution
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

int supervisor_cross::random_generator()
{
    std::random_shuffle ( v.begin(), v.end() );
    int random_variable= v[0];
    v.erase(v.begin()+0);
    return random_variable;
}

void supervisor_cross::init()
{
    ROS_INFO_STREAM("START");

    //********************* LOAD GRAPH **********************//

    typedef SmartDigraph Graph;
    typedef SmartDigraph::Node Node;
    typedef SmartDigraph::ArcMap<int> LengthMap;
    typedef SmartDigraph::NodeMap<double> DistMap;
    typedef SmartDigraph::NodeMap<double> CoordMap;
    typedef SmartDigraph::NodeMap<int> IdMap;
    typedef dim2::Point<double> Point;
    
    Graph g;
    LengthMap len(g);
    DistMap dist(g);
    CoordMap coord_x(g);
    CoordMap coord_y(g);
    IdMap id(g);
    SmartDigraph::NodeMap<Point> coords(g);
    
    try 
    {
	digraphReader(g, ros::package::getPath("robot_controller") + "/graph/prova_grafo.lgf").
	arcMap("length", len).
	nodeMap("coordinates_x",coord_x). 
	nodeMap("coordinates_y",coord_y).
	nodeMap("label",id).
	
	run();
    } 
    catch (Exception& error) 
    { 
	std::cerr << "Error: " << error.what() << std::endl;
	exit(1);
    }

    for (SmartDigraph::NodeIt n(g); n != INVALID; ++n) 
    {
	coord_x[n]=coord_x[n]*0.0381;
	coord_y[n]=coord_y[n]*0.0381;
	coords[n]=Point(coord_x[n], coord_y[n]);
    }
    
    ROS_INFO_STREAM("GRAPH LOADED");
    
    //********************* SPAWN ROBOTS **********************//
    
    stdr_robot::HandleRobot handler;
    std::srand(std::time(0));
        
    Node n0=g.nodeFromId(random_generator()); 
    Node n1=g.nodeFromId(random_generator()); 
    Node n2=g.nodeFromId(random_generator()); 
    Node n3=g.nodeFromId(random_generator());  
   
    if(ros::ok())
    {
	stdr_msgs::RobotMsg msg;
	std::string robot_type = ros::package::getPath("robot_controller") + "/robots/simple_robot.xml";

	for(int i = 0; i < 1; i++)
	{
	    try 
	    {
		msg = stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>(robot_type);
	    }
	    catch(stdr_parser::ParserException& ex)
	    {
		ROS_ERROR("[STDR_PARSER] %s", ex.what());
		exit(1);
	    }
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x = coord_x[n0]; 
	    msg.initialPose.y = coord_y[n0]; 
	    msg.initialPose.theta = 0 + 0.3; 
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
	    }
	    catch (stdr_robot::ConnectionException& ex) 
	    {
		ROS_ERROR("%s", ex.what());
		exit(1);
	    }
	}
	
	for(int i = 0; i < 1; i++)
	{
	    try 
	    {
		msg = stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>(robot_type);
	    }
	    catch(stdr_parser::ParserException& ex)
	    {
		ROS_ERROR("[STDR_PARSER] %s", ex.what());
		exit(1);
	    }
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x =coord_x[n1]; 
	    msg.initialPose.y =coord_y[n1]; 
	    msg.initialPose.theta = 0 + 0.3; 
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
	    }
	    catch (stdr_robot::ConnectionException& ex) 
	    {
		ROS_ERROR("%s", ex.what());
		exit(1);
	    }
	}
	
	for(int i = 0; i < 1; i++)
	{
	    try 
	    {
		msg = stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>(robot_type);
	    }
	    catch(stdr_parser::ParserException& ex)
	    {
		ROS_ERROR("[STDR_PARSER] %s", ex.what());
		exit(1);
	    }
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x = coord_x[n2];
	    msg.initialPose.y = coord_y[n2];
	    msg.initialPose.theta = M_PI + 0.3; 
	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
	    }
	    catch (stdr_robot::ConnectionException& ex) 
	    {
		ROS_ERROR("%s", ex.what());
		exit(1);
	    }
	}

	for(int i = 0; i < 1; i++)
	{
	    try 
	    {
		msg = stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>(robot_type);
	    }
	    catch(stdr_parser::ParserException& ex)
	    {
		ROS_ERROR("[STDR_PARSER] %s", ex.what());
		exit(1);
	    }
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x =  coord_x[n3]; 
	    msg.initialPose.y =  coord_y[n3]; 
	    msg.initialPose.theta = M_PI + 0.3; 	
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
	    }
	    catch (stdr_robot::ConnectionException& ex) 
	    {
		ROS_ERROR("%s", ex.what());
		exit(1);
	    }
	}
	ros::spinOnce();
    }
    
    ROS_INFO_STREAM("ROBOTS SPAWNED");

    //*********************INITIALIZATION**********************//
    
    pnh.param<int>("robot_number", n, 4);
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
	tmp.id = i;
	robots.push_back(tmp);
	
	ros::Publisher tmp_pub;
	tmp_pub = nh.advertise<geometry_msgs::Twist>("/" + tmp.robot_name + "/cmd_vel", 100); 
	controller_pubs.push_back(tmp_pub);
    }

    ReadPoses();
    
    ROS_INFO_STREAM("INITIALIZATION OK");

    
    //********************* COMPUTE PATH **********************//
    
    std::vector<Node> start;
    start.push_back(n0);
    start.push_back(n1);
    start.push_back(n2);
    start.push_back(n3);
    
    std::vector<Node> goals;
    goals.push_back(g.nodeFromId(7));
    goals.push_back(g.nodeFromId(9));
    goals.push_back(g.nodeFromId(16));
    goals.push_back(g.nodeFromId(11));
    
    ROS_WARN_STREAM("Initial Node ID robot0: "<< g.id(n0) << " ----> Goal Node ID robot0: 7");
    ROS_WARN_STREAM("Initial Node ID robot1: "<< g.id(n1) << " ----> Goal Node ID robot1: 9");
    ROS_WARN_STREAM("Initial Node ID robot2: "<< g.id(n2) << " ----> Goal Node ID robot2: 16");
    ROS_WARN_STREAM("Initial Node ID robot3: "<< g.id(n3) << " ----> Goal Node ID robot3: 11"); 
   
    for (int i = 0; i < start.size(); i++) 
    {
	Dijkstra<Graph, LengthMap> dijkstra_test(g,len);
	    
	dijkstra_test.run(start.at(i), goals.at(i));
	
	std::cout << "PATH Robot" << i << ": ";
	   
	if (dijkstra_test.dist(goals.at(i)) > 0)
	{   
	    for (Node v = goals.at(i); v != start.at(i); v = dijkstra_test.predNode(v)) 
	    {
		geometry_msgs::Pose2D tmp;
		tmp.y = coord_y[v];
		tmp.x = coord_x[v];	    
		
		robots[i].ref.push_back(tmp);
		
		std::cout << g.id(v) << " <- ";
	    }
	    std::cout << g.id(start.at(i))<< std::endl;

	    std::reverse(robots[i].ref.begin(),robots[i].ref.end());
	}
	else
	{
	    geometry_msgs::Pose2D tmp;
	    tmp.x = robots[i].curr_pose.x;
	    tmp.y = robots[i].curr_pose.y;	    
	    robots[i].ref.push_back(tmp); 
	}
    }
    
    ROS_INFO_STREAM("PATH COMPUTED");
    
    usleep(1000*1000);
}

void supervisor_cross::run()
{
    ros::Rate loop_rate(30);
    ROS_INFO_STREAM("RUN CONTROL");

    
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
	    	    	    
	    if(robots[i].err_lin < 1 && robots[i].ref.size()>1)
		robots[i].ref.erase(robots[i].ref.begin()+0); //SWITCH GOAL
	    
	    
	    //Robot i look at all the other robots j
	    for(int j = 0; j < n; j++)
	    {
		if(i != j)
		{
		    double gamma = atan2(robots[j].curr_pose.y - robots[i].curr_pose.y, robots[j].curr_pose.x - robots[i].curr_pose.x); //angle between horizontal and the rect connect i and j
		    double theta = robots[i].curr_pose.theta; //current orientation of i
		    double alpha= M_PI/5; //half vision angle    
		    double angle = fabs(fmod(theta - gamma, 2*M_PI)); 
		    
		    double dist = sqrt(pow(robots[i].curr_pose.x - robots[j].curr_pose.x,2) + pow(robots[i].curr_pose.y - robots[j].curr_pose.y,2)); //distance between i and j

// 		    if (i < j)
// 			double alpha = M_PI; //half vision angle
		    
		    if(fabs(sin(robots[i].err_ang)) > 0.05)
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
			    if(dist >= 6 && dist < 8)
			    {
				matrix.at(i).at(j) = state_transition::near_car;
			    }
			    
			    if(dist < 6)
			    {
				matrix.at(i).at(j) = state_transition::stop_now;
				
				if(robots[i].id > robots[j].id && dist > 3) 
				    matrix.at(i).at(j) = state_transition::road_free;
				if(robots[i].id > robots[j].id && dist < 3)
				    matrix.at(i).at(j) = state_transition::stop_now;
			    }

			}
			
/*			if(robots[i].curr_pose.x <= 14.5 && robots[i].curr_pose.x >= 13.5 &&  robots[i].curr_pose.y <= 17.5 && robots[i].curr_pose.y >= 16.5 || 
			    robots[i].curr_pose.x <= 23.5 && robots[i].curr_pose.x >= 22.5 && robots[i].curr_pose.y <= 14.5 && robots[i].curr_pose.y >= 13.5||
			    robots[i].curr_pose.x <= 26.5 && robots[i].curr_pose.x >= 25.5 && robots[i].curr_pose.y <= 23.5 && robots[i].curr_pose.y >= 22.5 ||
			    robots[i].curr_pose.x <= 17.5 && robots[i].curr_pose.x >= 16.5 && robots[i].curr_pose.y <= 26.5 && robots[i].curr_pose.y >= 25.5 )
			{  
			    alpha*=2; //half vision angle    
			    
			    if(angle < alpha && dist < 13)
				if(i > j)
				    matrix.at(i).at(j) = state_transition::road_free;
				else
				    matrix.at(i).at(j) = state_transition::stop_now;
			}	*/	    		    
		    }
		}
	    }
	    
	    if(robots[i].err_lin < 0.05 && robots[i].ref.size() == 1) //DELETE ROBOT
	    {
		stdr_robot::HandleRobot handler;
		std::string name("robot" + std::to_string(robots[i].id));
		try
		{    
		    if (handler.deleteRobot(name))
		    {
			ROS_INFO("Robot %s deleted successfully", name.c_str());
		    }
		    else 
		    {
			ROS_ERROR("Could not delete robot %s", name.c_str());
		    }
		}
		catch (stdr_robot::ConnectionException& ex) 
		{
		    ROS_ERROR("%s", ex.what());
		}
		robots.erase(robots.begin() + i);
		n--;
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
// 		robots[i].twist.linear.x = 0.05*robots[i].err_lin; // MULTI CROSS
		robots[i].twist.linear.x = 1*robots[i].err_lin;  // SINGLE CROSS
		if (robots[i].twist.linear.x > 7)
		    robots[i].twist.linear.x = 7;
		if (robots[i].twist.linear.x < 0.1)
		    robots[i].twist.linear.x = 0.1;
	    }
	    if(robots[i].state == state_machine_STATE::MOVE_SLOW)
	    { 
		robots[i].twist.angular.z = 3*sin(robots[i].err_ang);
// 		robots[i].twist.linear.x = 0.02*robots[i].err_lin; // MULTI CROSS
		robots[i].twist.linear.x = 0.2*robots[i].err_lin;  // SINGLE CROSS
		if (robots[i].twist.linear.x > 2)
		    robots[i].twist.linear.x = 2;
		if (robots[i].twist.linear.x < 0.05)
		    robots[i].twist.linear.x = 0.05;		
	    }
	    if(robots[i].state == state_machine_STATE::STOP)
	    {
		robots[i].twist.angular.z = 0.0;
		robots[i].twist.linear.x = 0.0;	
	    }
	    controller_pubs[robots[i].id].publish(robots[i].twist);
	}

	ros::spinOnce();
	loop_rate.sleep();
    }
}

