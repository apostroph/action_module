/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define PERIOD 10

#include "actionModule.h"


actionModule::actionModule(int argc, char** argv):
request(false)
{    
	ros::init(argc, argv, "action_node");
	
	//namedWindow("Vision and states", CV_WINDOW_AUTOSIZE+WINDOW_OPENGL);
	n = ros::NodeHandle("~");
	spinner = new ros::AsyncSpinner(1);
	spinner->start();
	
// 	groupRight = new moveit::planning_interface::MoveGroup("right_arm");
// 	groupLeft = new moveit::planning_interface::MoveGroup("left_arm");
// 	planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
	
	// Create a ROS publisher for the output point cloud
	pubHand = n.advertise<pr2_controllers_msgs::Pr2GripperCommand> ("/r_gripper_controller/command", 1);
	
	pubActObj = n.advertise<pr2_pbd_interaction::Vision> ("/action/objects", 1);
	pubActExec = n.advertise<pr2_pbd_speech_recognition::Command> ("/action/perform_action", 1);
	
	subDBN = n.subscribe ("/DBN/states_list", 1, &actionModule::fromDBN, this);
	
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
	
	bool found = false;
	//Search if policy already receved in the past
	for(auto policy: current_policies){
		if(policy.cmd.compare(actionType) == 0){
			found = true;
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
	
	
	
// 	if(current_cmd.compare(actionType) != 0){
// 		current_cmd = actionType;
// 		ROS_INFO("New action detected");
// 		begin = ros::Time::now();
// 		end = begin;
// 	}
	

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

string actionModule::chooseAction(string actionType, double x, double y, double z, double xf, double yf, double zf){
	string action_cmd = "";
	if(actionType.find("putOn") != std::string::npos){
		action_cmd += "7=>";
		action_cmd += pre_grasp(x, y, z); //a bit back from the object
		action_cmd += graps(x, y, z); //close hand on object
		action_cmd += post_grasp(x, y, z); //rize the hand in the air over x, y, z
		action_cmd += post_grasp(xf, yf, zf); //rize the hand in the air over x, y, z
		action_cmd += release(xf, yf, zf); //release the object at x, y, z
		action_cmd += pre_grasp(xf, yf, zf); //a bit back from the object
	}else if(actionType.find("push")){
	  
	}
	action_cmd += init_pos();
	
	return action_cmd;
} 

string actionModule::pre_grasp(double x, double y, double z){
	string cmd;
	cmd += to_string(x-0.15) + ","; //x
	cmd += to_string(y) + ","; //y
	cmd += to_string(z) + ","; //z
	cmd += to_string(0) + ","; //rx
	cmd += to_string(0) + ","; //ry
	cmd += to_string(0) + ","; //rz
	cmd += to_string(1) + ",";//hand open
	cmd += to_string(2) + ";";//wait 1 sec
	
	return cmd;
}

string actionModule::graps(double x, double y, double z){
	string cmd;
	cmd += to_string(x-0.1) + ","; //x
	cmd += to_string(y) + ","; //y
	cmd += to_string(z) + ","; //z
	cmd += to_string(0) + ","; //rx
	cmd += to_string(0) + ","; //ry
	cmd += to_string(0) + ","; //rz
	cmd += to_string(0) + ",";//hand closed
	cmd += to_string(2) + ";";//wait 1 sec
	
	return cmd;
}

string actionModule::post_grasp(double x, double y, double z){
	string cmd;
	cmd += to_string(x) + ","; //x
	cmd += to_string(y) + ","; //y
	cmd += to_string(z+0.15) + ","; //z
	cmd += to_string(0) + ","; //rx
	cmd += to_string(0) + ","; //ry
	cmd += to_string(0) + ","; //rz
	cmd += to_string(0) + ",";//hand closed
	cmd += to_string(2) + ";";//wait 1 sec  
	
	return cmd;
}

string actionModule::release(double x, double y, double z){
	string cmd;
	cmd += to_string(x) + ","; //x
	cmd += to_string(y) + ","; //y
	cmd += to_string(z+0.05) + ","; //z
	cmd += to_string(0) + ","; //rx
	cmd += to_string(0) + ","; //ry
	cmd += to_string(0) + ","; //rz
	cmd += to_string(1) + ",";//hand open
	cmd += to_string(2) + ";";//wait 1 sec
	
	return cmd;
}

string actionModule::init_pos(){
	string cmd;
	cmd += to_string(0.5) + ","; //initial x
	cmd += to_string(-0.3) + ","; //initial y
	cmd += to_string(0.8) + ","; //initial z
	cmd += to_string(0) + ","; //rx
	cmd += to_string(0) + ","; //ry
	cmd += to_string(0) + ","; //rz
	cmd += to_string(1) + ",";//hand open
	cmd += to_string(2) + ";";//wait 1 sec
	
	return cmd;  
}

void actionModule::executeAction(const string action_def){
	vector<actUtils::handPose> listActions;
	
	listActions = parseAction(action_def);
	float qx, qy, qz, qw;
	getQuaternion(qx, qy, qz, qw, M_PI/2);
	
	for(auto primitive:listActions){
		if(primitive.openHand == 1){
			pr2_controllers_msgs::Pr2GripperCommand openHand;
				openHand.position = 0.08;
				openHand.max_effort = -1;
			pubHand.publish (openHand);
		}else{
			pr2_controllers_msgs::Pr2GripperCommand openHand;
				openHand.position = 0.005;
				openHand.max_effort = -1;
			pubHand.publish (openHand);
		}
	  
		ros::Duration dH = ros::Duration(0.5, 0);
		dH.sleep();
	  
		geometry_msgs::Pose target_pose;
		target_pose.orientation.x = qx;
		target_pose.orientation.y = qy;
		target_pose.orientation.z = qz;
		target_pose.orientation.w = qw;
		
		target_pose.position.x = primitive.x;
		target_pose.position.y = primitive.y;
		target_pose.position.z = primitive.z;
		
// 		groupRight->setRandomTarget();
		// plan the motion and then move the group to the sampled target 
		groupRight->setPoseTarget(target_pose);
		groupRight->move();
		
		ros::Duration d = ros::Duration(primitive.pause, 0);
		d.sleep();
	}
  
}

std::vector<actUtils::handPose> actionModule::parseAction(const string action_def){
	string line = action_def;
	std::vector<actUtils::handPose> listOfPrimitives;
	
	int nbPrimitive = stoi(line.substr(0, line.find("=>")));
	
	line = line.substr(line.find("=>") +2, -1);
	for(int count = 0; count < nbPrimitive; count++){
		actUtils::handPose newPose;
		newPose.x = stof(line.substr(0, line.find(","))); line = line.substr(line.find(",")+1, -1);
		newPose.y = stof(line.substr(0, line.find(","))); line = line.substr(line.find(",")+1, -1);
		newPose.z = stof(line.substr(0, line.find(","))); line = line.substr(line.find(",")+1, -1);
		newPose.rx = stof(line.substr(0, line.find(","))); line = line.substr(line.find(",")+1, -1);
		newPose.ry = stof(line.substr(0, line.find(","))); line = line.substr(line.find(",")+1, -1);
		newPose.rz = stof(line.substr(0, line.find(","))); line = line.substr(line.find(",")+1, -1);
		newPose.openHand = stoi(line.substr(0, line.find(","))); line = line.substr(line.find(",")+1, -1);
		newPose.pause = stof(line.substr(0, line.find(";"))); line = line.substr(line.find(";")+1, -1);
		
// 		cout<<newPose.x<<" :: "<<newPose.y<<" :: "<<newPose.z<<"\n\t>> "<<newPose.rx<<" :: "<<newPose.ry<<" :: "<<newPose.rz<<"\n\t>> "<<newPose.openHand<<" :: "<<newPose.pause<<endl;
		listOfPrimitives.push_back(newPose);
	}
	
	return listOfPrimitives;  
}
void actionModule::getQuaternion(float &qx, float &qy, float &qz, float &qw, float angle){

	float m00 = 1; float m01 = 0; float m02 = 0;
	float m10 = 0; float m11 = cos(angle); float m12 = -sin(angle);
	float m20 = 0; float m21 = sin(angle); float m22 = cos(angle);
	
	float tr = m00 + m11 + m22;

	if (tr > 0) { 
		float S = sqrt(tr+1.0) * 2; // S=4*qw 
		qw = 0.25 * S;
		qx = (m21 - m12) / S;
		qy = (m02 - m20) / S; 
		qz = (m10 - m01) / S; 
	} else if ((m00 > m11)&(m00 > m22)) { 
		float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
		qw = (m21 - m12) / S;
		qx = 0.25 * S;
		qy = (m01 + m10) / S; 
		qz = (m02 + m20) / S; 
	} else if (m11 > m22) { 
		float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		qw = (m02 - m20) / S;
		qx = (m01 + m10) / S; 
		qy = 0.25 * S;
		qz = (m12 + m21) / S; 
	} else { 
		float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		qw = (m10 - m01) / S;
		qx = (m02 + m20) / S;
		qy = (m12 + m21) / S;
		qz = 0.25 * S;
	}
}


void actionModule::callbackPR2_cmd(const std_msgs::String msg){
	
}
/* Called periodically every getPeriod() seconds */
bool actionModule::loop() {
    
  //loop rate at 30FPS max
    ros::Rate loop_rate(20);
    
    while(ros::ok()){
	
      
	ros::spinOnce();
/*	
	
	pr2_controllers_msgs::Pr2GripperCommand openHand;
		openHand.position = 0.08;
		openHand.max_effort = -1;
	pubHand.publish (openHand);
	
	executeAction("1=>0.618, -0.023, 0.80, 0, 0, 0, 0, 2");
	executeAction("1=>0.70, -0.023, 0.80, 0, 0, 0, 0, 2");
	
		openHand.position = 0.01;
		openHand.max_effort = -1;
	pubHand.publish (openHand);
	
	executeAction("1=>0.618, -0.023, 0.80, 0, 0, 0, 0, 2");*/
// 	string cmd = chooseAction("putOn", 0.6286962032318115, 0.006107705179601908, 0.7632627248764038, 0.6286962032318115, 0.006107705179601908, 0.7632627248764038);
// 	executeAction(cmd);

// 	pr2_pbd_interaction::Vision cmd;
// 	cmd.command = cmd.EXECUTE_ACTION;
// 	cmd.action_id = 0;
	
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
			
			//Put that in another loop, so we can execute the closest action in case two policies have the same Ts
			if(T1 > 0.8 || T3 == 1){
			// 	
			// 	executeAction(action_cmd);
				pr2_pbd_interaction::Vision cmd_obj;
				
				for(auto cluster: msg_temp.clusters){
					cmd_obj.clusters.push_back(cluster);
					cout<<"+";
				}

				pubActObj.publish(cmd_obj);
				ROS_INFO("Objects sent");
				
				ros::Duration(2).sleep();
				
				pr2_pbd_speech_recognition::Command cmd;
				cmd.command = cmd.EXECUTE_ACTION;
				cmd.acton_id = 1;

				pubActExec.publish(cmd);
				ROS_INFO("%s sent", _cmd.data());
				
				current_policies.clear();
			}
			
			//Visualization helping
			cv::putText(policyVisualization, _cmd, cv::Point(15, 15+count*20), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
			rectangle(policyVisualization, Point(400+count*40, 350), Point(435+count*40, 1+350-350*T1), Scalar(50*count , 50*count, 50*count), -1);
			
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

