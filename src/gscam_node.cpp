
#include <ros/ros.h>
#include <stdlib.h>
#include <gscam/gscamsolo.h>

int main (int argc, char **argv){
    //initialize the ros system
    ros::init(argc, argv,"gscam_solo");
    
    //Establish this program as a ROS node
    ros::NodeHandle nh;
    
    //Use a private node handle so that multiple instances of the node can
    // be run simultaneously while using different parameters.
    // Parameters defined in the .cfg file do not need to be initialized here
    // as the dynamic_reconfigure::Server does this for you.
    ros::NodeHandle nh_private("~");
  
    ROS_INFO_STREAM("Solo camera driver");
    
    gscamsolo::GSCamSolo solocamera_driver(nh,nh_private);
    
    solocamera_driver.run();
    
}
