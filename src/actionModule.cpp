/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define PERIOD 10

#include "actionModule.h"


actionModule::actionModule(int argc, char** argv):
request(false), ongoing_action(false)
{    
	ros::init(argc, argv, "action_node");
	
	//namedWindow("Vision and states", CV_WINDOW_AUTOSIZE+WINDOW_OPENGL);
	n = ros::NodeHandle("~");
	spinner = new ros::AsyncSpinner(1);
	spinner->start();
	
	pubActObj = n.advertise<pr2_pbd_interaction::Vision> ("/action/objects", 1);
	pubActExec = n.advertise<pr2_pbd_speech_recognition::Command> ("/action/perform_action", 1);
	
	subDBN = n.subscribe ("/DBN/states_list", 1, &actionModule::fromDBN, this);
	
	action_feedback = n.subscribe ("/action_state_feedback", 1, &actionModule::fromBackend, this);
	
	end = ros::Time::now();
	begin = ros::Time::now();
	
	msg_temp = RGB_pcl::States();
	
	//Link the escape comd to the sign out method
	signal(SIGINT, &actionModule::sigintHandler);
}

void actionModule::sigintHandler(int sig)
{
	ROS_INFO("Action module shutdown");
	ros::shutdown();
	
	exit(0);
}

void actionModule::fromDBN(const RGB_pcl::States msg){
// 	
	string actionType = msg.states[0]+"-"+msg.states[1];
	msg_temp = msg;
	
	if(!ongoing_action){
		bool found = false;
		//Search if policy already receved in the past
		for(auto count = 0; count < current_policies.size(); count++){
			if(current_policies[count].cmd.compare(actionType) == 0){
				found = true;
				current_policies[count].strenght += 0.1;
				if(current_policies[count].strenght > 1){
					current_policies[count].strenght = 1;
				}
				break;
			}
		}
		//If not found, memorize the policy
		if(!found){
			policy new_policy;
			new_policy.cmd = actionType;
			new_policy.starting_time = ros::Time::now();
			new_policy.strenght = 0.5;
			cout<<new_policy.cmd<<" ==> added"<<endl;
			current_policies.push_back(new_policy);
		}
	}
}


void actionModule::fromBackend(const std_msgs::String msg){
	ongoing_action = false;
}


//T1 increase as the elapsed_time gets higher than the estimated period T
inline double actionModule::get_T1(const double elapsed_time){
	double T1 = 0;
	if(elapsed_time > PERIOD){
		T1 = ((PERIOD-(2*PERIOD-elapsed_time))/PERIOD);			  
	}
	return T1;
}

//T2 gets higher when the progress is negative
inline double actionModule::get_T2(const double distance_){
	double T2 = 0;
	if(distance_ < 0){
		T2 = 1;			  
	}
	return T2;
}

//T3 increases when user ask for help
inline double actionModule::get_T3(const bool request_){
	double T3 = 0;
	if(request_ != 0){
		T3 = 1;			  
	}
	return T3;
}

//T4 increases when several policies have the same outcome
inline double actionModule::get_T4(const double distance_){
	double T4 = 0;
	if(distance_ > 0){
		T4 = 1;
	}
	return T4;
}

void actionModule::callbackPR2_cmd(const std_msgs::String msg){
	
}
/* Called periodically every getPeriod() seconds */
bool actionModule::loop() {
    
  //loop rate at 30FPS max
    ros::Rate loop_rate(20);
    
    while(ros::ok()){
	
      
	ros::spinOnce();
	
	int nbP = current_policies.size();
	int size = 0;
	if(nbP > 10){
		size = nbP-10;
	}
	policyVisualization = cv::Mat(Size(600+(20*size), 400+(40*size)), CV_8UC3);
	policyVisualization = Scalar(255, 255, 255);
	
	
	//Separation lines
	line(policyVisualization, Point(380, 400), Point(380, 0), Scalar(0, 0, 0), 2);
	line(policyVisualization, Point(380, 400), Point(600+(40*size), 400), Scalar(0, 0, 0), 2);
	
	//graph lines
	line(policyVisualization, Point(395, 350), Point(395, 50), Scalar(0, 0, 0), 1);
	line(policyVisualization, Point(395, 350), Point(550+(40*size), 350), Scalar(0, 0, 0), 1);
	
	//Threshold line
	line(policyVisualization, Point(380, 350-0.8*350), Point(600+(40*size), 350-0.8*350), Scalar(0, 150, 0), 2);
// 	
	//estimate the motivaton signal and delete old policies
	for(auto count = 0; count < current_policies.size(); count++){
		double elapsed_time;
		end = ros::Time::now();
		elapsed_time = (double)(end.toSec()-current_policies[count].starting_time.toSec());
		if(current_policies[count].strenght <= 0 || elapsed_time > 2*PERIOD){
			cout<<current_policies[count].cmd<<" ==> deleted"<<endl;
			current_policies.erase(current_policies.begin()+count);
			count--;
		}else{
			double T1, T2, T3, T4;
			//T1 increase as the elapsed_time gets higher than the estimated period T
			//T2 gets higher when the progress is negative
			//T3 increases when user ask for help
			//T4 increases when several policies have the same outcome
			
			//T1 based on time and elapsed_time
			T1 = get_T1(elapsed_time);
			
			//T2 based on progress
			T2 = get_T2(false);
			
			//T3 based on user request
			T3 = get_T3(request);
			
			//T4 based on progress
			T4 = get_T4(0);	
			
			string _cmd = current_policies[count].cmd;
			
			current_policies[count].strenght -= 0.001;
			
			//Put that in another loop, so we can execute the closest action in case two policies have the same Ts
			if(T1 > 0.8 || T3 == 1){
			// 	
			// 	executeAction(action_cmd);
				pr2_pbd_interaction::Vision cmd_obj;
				
				for(auto cluster: msg_temp.clusters){
					cmd_obj.clusters.push_back(cluster);
				}

				pubActObj.publish(cmd_obj);
				cout<<msg_temp.clusters.size()<<" objects sent"<<endl;
				
				ros::Duration(2).sleep();
				
				pr2_pbd_speech_recognition::Command cmd;
				cmd.command = cmd.EXECUTE_ACTION;
				int action_pos = _cmd.find("action");
				int action_nb = stoi(_cmd.substr(action_pos+6, action_pos+7));
				
				int right = 0;
				if(msg_temp.y[0] < 0){
					right = 1;
				}
				cmd.acton_id = 2*action_nb+right;

				pubActExec.publish(cmd);
				cout<< _cmd.data()<<" sent. Action: "<<action_nb<<endl;
				
				current_policies.clear();
				
				ongoing_action = true;
			}
			
			//Visualization helping
			cv::putText(policyVisualization, _cmd, cv::Point(15, 15+count*20), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
			rectangle(policyVisualization, Point(400+count*40, 350), Point(435+count*40, 1+350-350*current_policies[count].strenght), Scalar(50*count , 50*count, 50*count), -1);
			
		}	  
	}
	
	imshow("Visualization", policyVisualization);

	
	loop_rate.sleep();
	
	end = ros::Time::now();
	
	request = false;
	int k = cv::waitKey(1);
	if(k == 32){
		request = true;
	}
    }
    ros::shutdown();
	
    return true;
}

actionModule::~actionModule(){
	cout<<"Shutdown"<<endl;
}

