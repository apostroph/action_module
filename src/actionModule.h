/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#ifndef _ACTION_H_
#define _ACTION_H_

#include <string>

#include <iostream>
#include <iomanip>

#include "std_msgs/String.h"
#include <signal.h>

#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>

#include "actionUtils.h"
#include "RGB_pcl/States.h"
#include "pr2_pbd_interaction/Vision.h"
#include "pr2_pbd_speech_recognition/Command.h"

using namespace std;
using namespace cv;

struct policy{
	string cmd;
	ros::Time starting_time;
	double strength;	
	int distance;  
	RGB_pcl::States state_msg;
};

class actionModule{

public:
    /** 
     * document your methods too.
     */
    actionModule(int argc, char** argv);
    ~actionModule();
    
    bool loop();

private:
  
  ros::NodeHandle n;
  ros::AsyncSpinner *spinner;
  ros::Subscriber action_feedback;
  ros::Subscriber subDBN;
  
  ros::Publisher pubActObj;
  ros::Publisher pubActExec;
  
  //ROS quit handler
  static void sigintHandler(int sig);
  //claaback methods
  void callbackPR2_cmd(const std_msgs::String msg);
  void fromDBN(const RGB_pcl::States msg);
  void fromBackend(const std_msgs::String msg);
  
  vector<policy> current_policies;
  Mat policyVisualization;
  
  ros::Time begin, end;
  bool request;
  bool ongoing_action;
  int condition;
  
  //Motivational signals
  inline double get_T1(const double elapsed_time);
  inline double get_T2(const double distance_);
  inline double get_T3(const bool request_);
  inline double get_T4(const double distance_);
  
};


#endif // __ACTION_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

