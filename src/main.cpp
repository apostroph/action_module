/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "actionModule.h"
//#include "planeDetection/planeDetection.h"

int main(int argc, char ** argv) {
    
    ros::init(argc, argv, "action_node");
    
    
    ROS_INFO("Ready start the action module");
    
    /* create your service */    
    actionModule module(argc, argv); 
    module.loop();

    return 0;
}

