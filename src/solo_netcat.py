#!/usr/bin/env python  
import os 
import rospy

if __name__ == '__main__':
    rospy.init_node('DroneAPP')
    rospy.loginfo('Drone APP connected')
    #os.system('nc 10.1.1.1 5502')
    rospy.loginfo('Drone APP: bye!')
    
