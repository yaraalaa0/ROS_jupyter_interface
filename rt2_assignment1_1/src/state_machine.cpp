#include "ros/ros.h"
#include "rt2_assignment1_1/Command.h"
#include "rt2_assignment1_1/Position.h"
#include "rt2_assignment1_1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "rt2_assignment1_1/GoToPointAction.h"
#include "rt2_assignment1_1/GoToPointGoal.h"
#include "rt2_assignment1_1/GoToPointResult.h"
#include "rt2_assignment1_1/GoToPointFeedback.h"


bool start = false;

bool user_interface(rt2_assignment1_1::Command::Request &req, rt2_assignment1_1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1_1::RandomPosition>("/position_server");
   
   /*
   Initializing the action client 
   subscribing to the action service "/go_to_point_ac"
   */
   actionlib::SimpleActionClient<rt2_assignment1_1::GoToPointAction> client_act("/go_to_point_ac", true);
   
   /*
   Waiting for the action server to start
   */
   std::cout << "Waiting for action server" << std::endl;
   client_act.waitForServer();
   
   
   rt2_assignment1_1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   /*
   Initialize the variables to send the goal to the server and get the result
   */
   rt2_assignment1_1::GoToPointGoal goal;
   rt2_assignment1_1::GoToPointResultConstPtr res;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   	        /*
                get the random position and store it in the goal variable
                then, send it to the action server
                */
   		client_rp.call(rp);		
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << "  theta = " <<goal.theta << std::endl;
   		client_act.sendGoal(goal);
   		
   		/*
                Get the current result of achieving the goal from the server
                */
   		res = client_act.getResult();
   		ros::Rate r(1);
   		/*
                Loop until the robot reaches the goal 
                */
   		while((res->reached)== false ){
   		    /*
                    check in each loop if the goal is cancelled by the user.
                    If the goal is cancelled, send cancel goal request to the server
                    Then, break the loop
                */
   		    if(!start){
   		        std::cout << "\nGoal Cancelled" << std::endl;
   		        client_act.cancelGoal();
   		        break;
   		    }
   		    ros::spinOnce(); //checking for any callbacks needed
   		    r.sleep();
   		    res = client_act.getResult();
   		    
   		}
   		
   		if(res->reached){
   		std::cout << "Position reached" << std::endl;
   		}
   	}
   }
   return 0;
}
