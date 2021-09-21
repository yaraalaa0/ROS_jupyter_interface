/**
 * \file position_service.cpp
 * \brief this file is an implementation of the random position server
 * \author Carmine Recchiuto
 * \version 0.1
 * \date 20/09/2021
 *
 * \details
 *
 * Services: <BR>
 *  . /position_server
 *
 * Description :
 *
 * This node implements the random position server. When the server receives a request of 
 * minimum and maximum numbers, it replies with three random position values for x, y, and 
 * theta between the minimum and maximum limits. 
 *
 *
*/

#include "ros/ros.h"
#include "rt2_assignment1_1/RandomPosition.h"

/**
 * \brief this function generates a random number within a range
 * \param M defines the lower bownd of the range
 * \param N defines the upper bound of the range
 *
 * \return random number of type double within the specified range
 *
 * This function takes as inout the min and max limits of the range and uses the rand() 
 * function to generate a random number within the range 
*/
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * \brief this is the callback function of the random position server
 * \param req the service request defined in RandomPosition.srv file in the srv folder of the package
 * \param res the service response passed by reference to be filled by the callback function
 *
 * \return always true 
 *
 * This function receives a request that contains min and max bounds for both x and y positions.
 * It calls randMToN() function to generate a random number within the requested ranges for x and y.
 * For theta, it calls randMToN() with min bound of -3.14 (-pi), and max bound of 3.14 (+pi)
*/
bool myrandom (rt2_assignment1_1::RandomPosition::Request &req, rt2_assignment1_1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 * \brief this is main function of the node
 *
 * It initializes the node handle and the /position_server service, 
 * and assigns myrandom() as a callback function to the service
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
