## @package rt2_assignment1_1
# \file user_interface.py
# \brief this file is an implementation of the user interface node
# \author Carmine Recchiuto
# \version 0.1
# \date 20/09/2021
#
# \details
#
# Clients: <BR>
#  . /user_interface
#
#
# Description :
#
# This node implements the user interface. 
# It receives from the user a start or stop command, and sends it to the /user_interface server
#

import rospy
import time
from rt2_assignment1_1.srv import Command

##
# \brief this is the main function of the node
#
# It initializes the node handle and the client to the service /user_interface
# It waits for an input from the user of 1 or 0 to start the robot or stop it, respectively.
# If the user inputs 1, it sends a "start" request to the /user_interface service
# If the user inputs 0, it sends a "stop" request to the /user_interface service
#
def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("The robot will stop now")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
