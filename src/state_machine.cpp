/**
 * \file state_machine.cpp
 * \brief this file is an implementation of the state machine node
 * \author Carmine Recchiuto
 * \version 0.1
 * \date 20/09/2021
 *
 * \details
 *
 * Services: <BR>
 *    /user_interface
 *
 * Clients: <BR>
 *   /position_server
 *  
 *   /go_to_point_ac
 *
 * Description :
 *
 * This node implements the state machine. When the user sends a start command, the node gets a random 
 * position (x,y, theta) using the /position_server. 
 * Then, it sends this position as a goal to /go_to_point_ac action service. While the robot hasn't 
 * reached the goal yet, it continues to check if the user has sent a cancel command. If yes, it 
 * sends a cancel request to the action service /go_to_point_ac
 * When the robot reaches the goal, it perceives another random goal
 *
 *
*/

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

/**
 * \brief this is the callback function of /user_interface service
 * \param req the service request defined in Command.srv file in the srv folder of the package. It contains a srting of "start" or "stop"
 * \param res the service response passed by reference to be filled by the callback function
 *
 * \return always true
 *
 * This function takes as input the command request of type string. It can be either "start" or "stop".
 * It fills the global variable start according to the received command.  
*/
bool user_interface(rt2_assignment1_1::Command::Request &req, rt2_assignment1_1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


/**
 * \brief this is the main function of the node
 *
 *
 * It initializes the node handle, the server of the /user_interface service, 
 * the client to /position_server , and the action client to /go_to_point_ac
 * It continuously checks for the value of the global variable start. If it is true, the node gets a 
 * random position by calling /position_server with x_min and y_min = -5.0 and x_max and y_max = 5.0 
 * Then, it sends the received random position as a new goal to the action service /go_to_point_ac
 * While the robot hasn't reached the goal yet, it continues to check if the user has sent a cancel command. 
 * If yes, it sends a cancel request to the action service /go_to_point_ac
 * When the robot reaches the goal, it goes over the same loop to go to another random goal.
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1_1::RandomPosition>("/position_server");
   
   actionlib::SimpleActionClient<rt2_assignment1_1::GoToPointAction> client_act("/go_to_point_ac", true);
   
   
   std::cout << "Waiting for action server" << std::endl;
   client_act.waitForServer();
   
   
   rt2_assignment1_1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   
   rt2_assignment1_1::GoToPointGoal goal;
   rt2_assignment1_1::GoToPointResultConstPtr res;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   	        
   		client_rp.call(rp);		
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << "  theta = " <<goal.theta << std::endl;
   		client_act.sendGoal(goal);
   		
   		
   		res = client_act.getResult();
   		ros::Rate r(1);
   		
   		while((res->reached)== false ){ ///< Loop until the robot reaches the goal 
   		    
                    ///< check in each loop if the goal is cancelled by the user.
                    ///< If the goal is cancelled, send cancel goal request to the server
                    ///< Then, break the loop
                
   		    if(!start){
   		        std::cout << "\nGoal Cancelled" << std::endl;
   		        client_act.cancelGoal();
   		        break;
   		    }
   		    ros::spinOnce();
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
