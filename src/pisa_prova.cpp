#include "pisa_prova.h"

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

double special_sin(double err_ang) 
{
    if(err_ang >= M_PI/2 && err_ang <= 3*M_PI/2 || err_ang <= -M_PI/2 && err_ang >= -3*M_PI/2)
    {
	return 1.0;
    }
    else
    {
	return fabs(sin(err_ang));
    }	    
}

pisa_prova::pisa_prova():pnh("~"),len(g),coord_x(g),coord_y(g),coords(g)  //Constructor
{
    n=0;
   
    for (int i=0; i<702; i++)
    {
	node.push_back(i);
    }
}

pisa_prova::~pisa_prova() //Desctructor
{
    
}

void pisa_prova::ReadPoses() //Read from tf the robots position
{
    for(int i = 0; i < n; i++)
    {
	tf::StampedTransform transform;
	try{
	    listener.waitForTransform("/world", robots[i].robot_name, ros::Time(0), ros::Duration(0) );
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

double pisa_prova::LinearErrY(geometry_msgs::Pose2D current, std::vector<geometry_msgs::Pose2D> reference)
{
    geometry_msgs::Pose2D reff = reference[0];
    return  reff.y - current.y;
}

double pisa_prova::LinearErrX(geometry_msgs::Pose2D current, std::vector<geometry_msgs::Pose2D> reference)
{
    geometry_msgs::Pose2D reff = reference[0];
    return  reff.x - current.x;
}

bool pisa_prova::evolve_state_machines(int i)	//State Machine evolution
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

int pisa_prova::random_generator(std::vector<int> v)
{
    std::random_shuffle (v.begin(), v.end());
    int random_variable= v[0];
    v.erase(v.begin()+0);
    return random_variable;
}

void pisa_prova::init()
{
    ROS_INFO_STREAM("START");
    
    pnh.param<int>("robot_number", n, 5);
    ROS_INFO_STREAM("Robot Number: " << n);

    //********************* LOAD GRAPH **********************//
    
    try 
    {
	digraphReader(g, ros::package::getPath("robot_controller") + "/graph/pisa.lgf").
	nodeMap("coordinates_x",coord_x). 
	nodeMap("coordinates_y",coord_y).
// 	nodeMap("label",id).
	
	run();
    } 
    catch (Exception& error) 
    { 
	std::cerr << "Error: " << error.what() << std::endl;
	exit(1);
    }

    for (SmartDigraph::NodeIt n(g); n != INVALID; ++n) 
    {
	coord_x[n]=coord_x[n];
	coord_y[n]=1861 - coord_y[n];
	coords[n]=Point(coord_x[n], coord_y[n]);
    }

    for (SmartDigraph::ArcIt a(g); a != INVALID; ++a) 
    {
	double distance = sqrt((pow((coord_x[g.source(a)] - coord_x[g.target(a)]),2.0))+(pow((coord_y[g.source(a)] - coord_y[g.target(a)]),2.0)));
	len[a] = distance;
    }
    
    ROS_INFO_STREAM("GRAPH LOADED");
       
    //********************* SPAWN ROBOTS **********************//
    
    stdr_robot::HandleRobot handler;
    std::srand(std::time(0));
    
    std::vector<Node> random_start_node;

    for(int i=0; i < n; i++)
    {
	random_start_node.push_back(g.nodeFromId(random_generator(node))); 
    }
    
    if(ros::ok())
    {
	stdr_msgs::RobotMsg msg;
	std::string robot_type = ros::package::getPath("robot_controller") + "/robots/simple_robot.xml";
		
	try 
	{
	    msg = stdr_parser::Parser::createMessage<stdr_msgs::RobotMsg>(robot_type);
	}
	catch(stdr_parser::ParserException& ex)
	{
	    ROS_ERROR("[STDR_PARSER] %s", ex.what());
	    exit(1);
	}
	
	for(int i = 0; i < random_start_node.size(); i++)
	{
	    double random_variable = std::rand()%7 +0.2 -M_PI;
	    msg.initialPose.x = coord_x[random_start_node[i]]; 
	    msg.initialPose.y = coord_y[random_start_node[i]]; 
	    msg.initialPose.theta = 0; 
	
	    stdr_msgs::RobotIndexedMsg namedRobot;
	
	    try 
	    {
		namedRobot = handler.spawnNewRobot(msg);
		ROS_INFO_STREAM("robot "<< i << " spawned");
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
    
    std::string name = "robot";
    
    for (int i = 0; i < n; i++)
    {
	Robot tmp;
	tmp.robot_name = name + std::to_string(i);
	tmp.err_ang = 0.0;
	tmp.err_lin = 0.0;
	tmp.state = state_machine_STATE::STOP;
	tmp.robot_state = 3;
	tmp.transition = "road_free";
	tmp.id = i;
	robots.push_back(tmp);
	
	ros::Publisher tmp_pub;
	tmp_pub = nh.advertise<robot_controller::robot>(tmp.robot_name + "/state", 1); 
		
	robot_pubs.push_back(tmp_pub);

// 	tmp_pub = nh.advertise<geometry_msgs::Twist>("/" + tmp.robot_name + "/cmd_vel", 100); 
// 	controller_pubs.push_back(tmp_pub);
    }

    ReadPoses();
    
    ROS_INFO_STREAM("INITIALIZATION OK");

    //********************* COMPUTE PATH **********************//

    std::vector<Node> random_goal_node;
    
    for(int i=0; i < n; i++)
    {
	random_goal_node.push_back(g.nodeFromId(random_generator(node))); 
    }

//     for(int i = 0; i < n; i++)
//     {
// 	ROS_WARN_STREAM("Initial Node ID robot " << i << ": "<< g.id(random_start_node.at(i)) << " ----> Goal Node ID robot " << i << ": " << g.id(random_goal_node.at(i)));
//     }
   
    for (int i = 0; i < random_start_node.size(); i++) 
    {
	Dijkstra<Graph, LengthMap> dijkstra_test(g,len);
	    
	dijkstra_test.run(random_start_node.at(i), random_goal_node.at(i));
	
// 	std::cout << "PATH Robot" << i << ": ";
	   
	if (dijkstra_test.dist(random_goal_node.at(i)) > 0)
	{   
	    for (Node v = random_goal_node.at(i); v != random_start_node.at(i); v = dijkstra_test.predNode(v)) 
	    {
		geometry_msgs::Pose2D tmp;
		tmp.y = coord_y[v];
		tmp.x = coord_x[v];	    
		
		robots[i].ref.push_back(tmp);
		robots[i].ref_node.push_back(v);
		
// 		std::cout << g.id(v) << " <- ";
	    }
// 	    std::cout << g.id(random_start_node.at(i))<< std::endl;
	    std::reverse(robots[i].ref.begin(),robots[i].ref.end());
	    std::reverse(robots[i].ref_node.begin(),robots[i].ref_node.end());
	}
	else
	{
	    geometry_msgs::Pose2D tmp;
	    tmp.x = robots[i].curr_pose.x;
	    tmp.y = robots[i].curr_pose.y;	    
	    robots[i].ref.push_back(tmp); 
	}

	robots[i].prev_ref_node = random_start_node[i];
    }
    
    ROS_INFO_STREAM("PATH COMPUTED");
        
    usleep(1000*1000);
}

void pisa_prova::run()
{   
    ros::Rate loop_rate(30);
    ROS_INFO_STREAM("RUN CONTROL");
    
    while (ros::ok())
    {
	ReadPoses();
	
	std::vector<state_transition> tmp(n, state_transition::road_free);
	std::vector<std::vector<state_transition>> matrix(n, tmp);	//matrix nxn containing state transition info (i,j) e (j,i) 
	std::vector<int> robot_num(g.arcNum(),0);
		
	for(int i = 0; i < n; i++)
	{
	    for (SmartDigraph::ArcIt a(g); a != INVALID; ++a) 
	    {
		if(robots[i].ref_node[0] == g.target(a) && robots[i].prev_ref_node == g.source(a))
		{
			if(g.id(a) == robots[i].prev_value) 
			    break;
			
			robot_num.at(g.id(a))++;
			
// 			ROS_INFO_STREAM("Numero Robot su arco " << g.id(a) << " = " << robot_num.at(g.id(a)));
// 			ROS_INFO_STREAM("Robot " << robots[i].id << " su arco " << g.id(a));
			robots[i].prev_value = g.id(a);
// 			len[a]+=;
		}
	    }
	    	    
	    double fx = LinearErrX(robots[i].curr_pose, robots[i].ref);
	    double fy = LinearErrY(robots[i].curr_pose, robots[i].ref);
	    
	    //Compute linear and angular error for robot i
	    robots[i].err_ang = atan2(fy,fx) - robots[i].curr_pose.theta;    
	    robots[i].err_lin = sqrt(pow(fx,2) + pow(fy,2));
	    	    	    
	    if(robots[i].err_lin < 1 && robots[i].ref.size() > 1)
	    {
		robots[i].prev_ref_node = robots[i].ref_node[0]; 
		robots[i].ref.erase(robots[i].ref.begin()+0); //SWITCH GOAL
		robots[i].ref_node.erase(robots[i].ref_node.begin()+0); //SWITCH GOAL
	    }
	    
	    //Robot i look at all the other robots j
	    for(int j = 0; j < n; j++)
	    {
		if(n==1)
		{
		    if(special_sin(robots[i].err_ang) > 0.05)
		    {
			matrix.at(i).at(j) = state_transition::rot_only;
		    }
		    else 
		    {
			matrix.at(i).at(j) = state_transition::move_rot;
		    }
		}
		    
		if(i != j && n != 1)
		{
		    double gamma = atan2(robots[j].curr_pose.y - robots[i].curr_pose.y, robots[j].curr_pose.x - robots[i].curr_pose.x); //angle between horizontal and the rect connect i and j
		    double theta = robots[i].curr_pose.theta; //current orientation of i
		    double alpha= M_PI/5; //half vision angle    
		    double angle = fabs(fmod(theta - gamma, 2*M_PI)); 
		    
		    double dist = sqrt(pow(robots[i].curr_pose.x - robots[j].curr_pose.x,2) + pow(robots[i].curr_pose.y - robots[j].curr_pose.y,2)); //distance between i and j
		    
		    if(special_sin(robots[i].err_ang) > 0.05)
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
			    if(dist >= 8 && dist < 10)
			    {
				matrix.at(i).at(j) = state_transition::near_car;
			    }
			    
			    if(dist < 8)
			    {
				matrix.at(i).at(j) = state_transition::stop_now;
				
				if(robots[i].id > robots[j].id && dist > 2) 
				    matrix.at(i).at(j) = state_transition::road_free;
				if(robots[i].id > robots[j].id && dist < 2)
				    matrix.at(i).at(j) = state_transition::stop_now;
			    }
			}   
		    }
		}
	    }
	    
	    if(robots[i].err_lin < 10 && robots[i].ref.size() == 1) //DELETE ROBOT
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
		    case state_machine_STATE::ROTATE_ONLY: /*ROS_WARN_STREAM("Robot " << std::to_string(i) << ": ROTATE_ONLY")*/;
			robots[i].robot_state = 0;
			break;
		    case state_machine_STATE::MOVE_AND_ROTATE: /*ROS_WARN_STREAM("Robot " << std::to_string(i) << ": MOVE_AND_ROTATE")*/;
			robots[i].robot_state = 1;
			break;
		    case state_machine_STATE::MOVE_SLOW: /*ROS_WARN_STREAM("Robot " << std::to_string(i) << ": MOVE_SLOW")*/; 
			robots[i].robot_state = 2;
			break;
		    case state_machine_STATE::STOP: /*ROS_WARN_STREAM("Robot " << std::to_string(i) << ": STOP")*/; 
			robots[i].robot_state = 3;
			break;
		    default: ROS_WARN_STREAM("FAIL"); break;
		}
	    }
	    
	    robot_controller::robot msg1;
	    
// 	    msg1.ref_x = robots[i].ref[0].x;
// 	    msg1.ref_y = robots[i].ref[0].y;
	    msg1.err_lin = robots[i].err_lin;
	    msg1.err_ang = robots[i].err_ang;
	    msg1.robot_state = robots[i].robot_state;
	    msg1.id = robots[i].id;
	    msg1.n = n;

	    robot_pubs[robots[i].id].publish(msg1);
	    
// 	    ROS_WARN_STREAM("ROBOT: " << robots[i].id << " pos x: " << robots[i].curr_pose.x << " pose y: " << robots[i].curr_pose.y);


// 	    if(robots[i].state == state_machine_STATE::ROTATE_ONLY)
// 	    {
// 		robots[i].twist.angular.z = 2*sin(robots[i].err_ang);
// 		robots[i].twist.linear.x = 0.0;	
// 	    }
// 	    if(robots[i].state == state_machine_STATE::MOVE_AND_ROTATE)
// 	    {
// 		robots[i].twist.angular.z = 2*sin(robots[i].err_ang);
// 		robots[i].twist.linear.x = 4*robots[i].err_lin; // MULTI CROSS
// 		if (robots[i].twist.linear.x > 10)
// 		    robots[i].twist.linear.x = 10;
// 		if (robots[i].twist.linear.x < 0.5)
// 		    robots[i].twist.linear.x = 0.5;
// 	    }
// 	    if(robots[i].state == state_machine_STATE::MOVE_SLOW)
// 	    { 
// 		robots[i].twist.angular.z = 2*sin(robots[i].err_ang);
// 		robots[i].twist.linear.x = 1.5*robots[i].err_lin; // MULTI CROSS
// 		if (robots[i].twist.linear.x > 7)
// 		    robots[i].twist.linear.x = 7;
// 		if (robots[i].twist.linear.x < 0.2)
// 		    robots[i].twist.linear.x = 0.2;		
// 	    }
// 	    if(robots[i].state == state_machine_STATE::STOP)
// 	    {
// 		robots[i].twist.angular.z = 2*sin(robots[i].err_ang);
// 		robots[i].twist.linear.x = 0.0;	
// 	    }
// 	    	    
// 	    controller_pubs[robots[i].id].publish(robots[i].twist);
	    ros::spinOnce();
	}
	ros::spinOnce();
	loop_rate.sleep();
    }
}

