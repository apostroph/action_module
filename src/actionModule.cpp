/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define PERIODE 5

#include "actionModule.h"


actionModule::actionModule(int argc, char** argv)
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
	
	//estimate the motivaton signal and delete old policies
	for(auto count = 0; count << current_policies.size(); count++){
		double elapsed_time;
		elapsed_time += (double)(end-current_policies[count].starting_time).toSec();
		
		if(current_policies[count].strenght <= 0 || elapsed_time > 500){
			current_policies.erase(current_policies.begin()+count);
			cout<<current_policies[count].cmd<<" ==> deleted"<<endl;
			count--;
		}else{
			end = ros::Time::now();
			double T1, T2, T3, T4;
			//T1 increase as the elapsed_time gets higher than the estimated period T
			//T2 gets higher when the progress is negative
			//T3 increases when user ask for help
			//T4 increases when several policies have the same outcome
			
			
		}	  
	}
	
// 	if(current_cmd.compare(actionType) != 0){
// 		current_cmd = actionType;
// 		ROS_INFO("New action detected");
// 		begin = ros::Time::now();
// 		end = begin;
// 	}
	
	if(0){
	// 	
	// 	executeAction(action_cmd);
		pr2_pbd_interaction::Vision cmd_obj;
		// 	cmd.command = cmd.EXECUTE_ACTION;
		// 	cmd.action_id = 0;
		
		for(auto cluster: msg.clusters){
			cmd_obj.clusters.push_back(cluster);
			cout<<"+";
		}

		pubActObj.publish(cmd_obj);
		ROS_INFO("Objects sent");
		
		pr2_pbd_speech_recognition::Command cmd;
		cmd.command = cmd.EXECUTE_ACTION;
		cmd.acton_id = 1;

		pubActExec.publish(cmd);
		ROS_INFO("Actions sent");
	}
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
// 	
	

	
	loop_rate.sleep();
	
	end = ros::Time::now();
    }
    ros::shutdown();
	
    return true;
}

actionModule::~actionModule(){
	cout<<"Shutdown"<<endl;
}

