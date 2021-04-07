#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include <vector>

#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/scoped_ptr.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


/*
   class RetrieveData{
   private:
   class callbackdata {
   public:
   void sub_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr) {
//std::cout<< "x: " << pose_msg_ptr->pose.position.x << std::endl;
//std::cout<< "y: " << pose_msg_ptr->pose.position.y << std::endl;
//std::cout<< "z: " << pose_msg_ptr->pose.position.z << std::endl;
configptr->pose.position.x = pose_msg_ptr->pose.position.x;
configptr->pose.position.y = pose_msg_ptr->pose.position.y;
configptr->pose.position.z = pose_msg_ptr->pose.position.z;
configptr->pose.orientation.x = pose_msg_ptr->pose.orientation.x;
configptr->pose.orientation.y = pose_msg_ptr->pose.orientation.y;
configptr->pose.orientation.z = pose_msg_ptr->pose.orientation.z;
configptr->pose.orientation.w = pose_msg_ptr->pose.orientation.w;
}
geometry_msgs::PoseStamped config;
geometry_msgs::PoseStamped* configptr = &config;
};
int Navg;
bool hasdata;	
geometry_msgs::PoseStamped avgConfig;
float sample_set_avg[7];
public:
RetrieveData(int Navg_) {
Navg = Navg_;	
hasdata = false;
}
void retrieve() {
ros::NodeHandle SUB;
callbackdata subdata;
std::cout<<"im alive"<<std::endl;
ros::Subscriber subscriber = SUB.subscribe("/vrpn_client_node/smallBox1/pose", 10, &callbackdata::sub_callback, &subdata);
float sample_set[7] = {0, 0, 0, 0, 0, 0, 0};
ros::Rate r(30);
ros::spinOnce();
int Navg_actual = 0;
for (int i=0; i<Navg; i++) {
ros::spinOnce();
if (subdata.configptr->pose.position.x == 0.0){
std::cout<<"Bad data"<<std::endl;
} else {
sample_set[0] += subdata.configptr->pose.position.x;                 
sample_set[1] += subdata.configptr->pose.position.y;
sample_set[2] += subdata.configptr->pose.position.z;
sample_set[3] += subdata.configptr->pose.orientation.x;
sample_set[4] += subdata.configptr->pose.orientation.y;
sample_set[5] += subdata.configptr->pose.orientation.z;
sample_set[6] += subdata.configptr->pose.orientation.w;
Navg_actual++;
//std::cout<<"hello brother"<< subdata.configptr->pose.position.x <<std::endl;
}
r.sleep();
if (!ros::ok()){
break;
}
}
for (int i=0; i<7; i++) {
sample_set_avg[i] = sample_set[i]/Navg_actual;
}
hasdata = true;
std::cout<<"im dying"<<std::endl;
}
geometry_msgs::PoseStamped* returnConfigPtr () {
if (!hasdata) {
std::cout<< "Call retrieve() before calling returnConfig()" <<std::endl;
} else {
	avgConfig.pose.position.x = sample_set_avg[0];
	avgConfig.pose.position.y = sample_set_avg[1];
	avgConfig.pose.position.z = sample_set_avg[2];
	avgConfig.pose.orientation.x = sample_set_avg[3];
	avgConfig.pose.orientation.y = sample_set_avg[4];
	avgConfig.pose.orientation.z = sample_set_avg[5];
	avgConfig.pose.orientation.w = sample_set_avg[6];
	return &avgConfig;
}
} 
};
*/

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_manipulator_node");
	ros::NodeHandle node_handle("~");
	//ros::AsyncSpinner spinner(1);
	//spinner.start();


	/////////////////////////////////////////////////////////////////////////
	static const std::string PLANNING_GROUP = "manipulator";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	move_group.setPlanningTime(40);
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
	const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();



	//LOADING A PLANNER
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	//from tutorial

	if (!node_handle.getParam("planning_plugin", planner_plugin_name))
		ROS_FATAL_STREAM("Could not find planner plugin name");
	try
	{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
					"moveit_core", "planning_interface::PlannerManager"));
	}
	catch (pluginlib::PluginlibException& ex)
	{
		ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
	}
	try
	{
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM("Using planner '" << planner_instance->getDescription() << "'");
	}

	catch (pluginlib::PluginlibException& ex)
	{
		const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (std::size_t i = 0; i < classes.size(); ++i)
			ss << classes[i] << " ";
		ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
				<< "Available plugins: " << ss.str());
	}
	/////////////////////////////////////////////////////////////////////////


	int Nobj=2;
	std::vector<moveit_msgs::CollisionObject> colObjVec;
	colObjVec.resize(Nobj);
	
	/*
	for (int i=0; i<Nobj; i++) {
		colObjVec[i].header.frame_id = "base_link";
	}
	*/

	colObjVec[0].header.frame_id = "base_link";
	colObjVec[1].header.frame_id = "ee_link";

	//
	colObjVec[0].id = "table";
	colObjVec[0].primitives.resize(1);
	colObjVec[0].primitives[0].type = colObjVec[0].primitives[0].BOX;
	colObjVec[0].primitives[0].dimensions.resize(3);
	colObjVec[0].primitives[0].dimensions[0] = 3;
	colObjVec[0].primitives[0].dimensions[1] = 3;
	colObjVec[0].primitives[0].dimensions[2] = 1;

	colObjVec[0].primitive_poses.resize(1);
	colObjVec[0].primitive_poses[0].position.x = 0;
	colObjVec[0].primitive_poses[0].position.y = 0;
	colObjVec[0].primitive_poses[0].position.z = -.5;
	colObjVec[0].primitive_poses[0].orientation.x = 0;
	colObjVec[0].primitive_poses[0].orientation.y = 0;
	colObjVec[0].primitive_poses[0].orientation.z = 0;
	colObjVec[0].primitive_poses[0].orientation.w = 1;
	colObjVec[0].operation = colObjVec[0].ADD;

	colObjVec[1].id = "eef";
	colObjVec[1].primitives.resize(1);
	colObjVec[1].primitives[0].type = colObjVec[0].primitives[0].BOX;
	colObjVec[1].primitives[0].dimensions.resize(3);
	colObjVec[1].primitives[0].dimensions[0] = .09;
	colObjVec[1].primitives[0].dimensions[1] = .1;
	colObjVec[1].primitives[0].dimensions[2] = .12;

	colObjVec[1].primitive_poses.resize(1);
	colObjVec[1].primitive_poses[0].position.x =.09*.5;
	colObjVec[1].primitive_poses[0].position.y = 0;
	colObjVec[1].primitive_poses[0].position.z = .02; 
	colObjVec[1].primitive_poses[0].orientation.x = 0;
	colObjVec[1].primitive_poses[0].orientation.y = 0;
	colObjVec[1].primitive_poses[0].orientation.z = 0;
	colObjVec[1].primitive_poses[0].orientation.w = 1;
	colObjVec[1].operation = colObjVec[0].ADD;





	planning_scene_interface.applyCollisionObjects(colObjVec);
	planning_scene_interface.addCollisionObjects(colObjVec);
	move_group.attachObject("eef","ee_link");
	/*
	   RetrieveData configData(50);
	   float readx, ready, readz;
	   while (ros::ok()){
	   configData.retrieve();
	   auto p = configData.returnConfigPtr(); //geometry_msgs::PoseStamped*
	   readx = -p->pose.position.y + .794;
	   ready = p->pose.position.x + .076;
	   readz = p->pose.position.z;
	   colObjVec[0].primitive_poses.resize(1);
	   colObjVec[0].primitive_poses[0].position.x = readx; 
	   colObjVec[0].primitive_poses[0].position.y = ready; 
	   colObjVec[0].primitive_poses[0].position.z = readz; 
	   colObjVec[0].primitive_poses[0].orientation.x = p->pose.orientation.x;
	   colObjVec[0].primitive_poses[0].orientation.y = p->pose.orientation.y;
	   colObjVec[0].primitive_poses[0].orientation.z = p->pose.orientation.z;
	   colObjVec[0].primitive_poses[0].orientation.w = p->pose.orientation.w;
	   colObjVec[0].operation = colObjVec[0].ADD;
	   planning_scene_interface.applyCollisionObjects(colObjVec);
	   planning_scene_interface.addCollisionObjects(colObjVec);
	   std::cout<< "x: " << -p->pose.position.y << std::endl;	
	   std::cout<< "y: " << p->pose.position.x << std::endl;	
	   std::cout<< "z: " << p->pose.position.z << std::endl;
	   */
	move_group.setEndEffectorLink("ee_link");
	geometry_msgs::Pose initpose;
	moveit::planning_interface::MoveGroupInterface::Plan initplan;



	float posmat[3][3];
	posmat[0][0] = .4;
	posmat[0][1] = .4;
	posmat[0][2] = .4; 
	posmat[1][0] = .4; 
	posmat[1][1] = -.4;
	posmat[1][2] = .6;
	posmat[2][0] = .4; 
	posmat[2][1] = 0; 
	posmat[2][2] = .8; 



	while (ros::ok()){
		for (int i=0; i<3; i++){
			initpose.position.x = posmat[i][0];
			initpose.position.y = posmat[i][1];
			initpose.position.z = posmat[i][2];
			/*
			tf2::Quaternion initorient;
			initorient.setRPY(0, 0, 0);
			initpose.orientation = tf2::toMsg(initorient); 
			*/
			initpose.orientation.x = 1;
			initpose.orientation.y = 0;
			initpose.orientation.z = 0;
			initpose.orientation.w = 0;
			move_group.setStartStateToCurrentState();
			move_group.setPoseTarget(initpose);
			move_group.setPlanningTime(5.0);
			
			for (int ii=0; ii<4; ii++){
				move_group.plan(initplan);
				bool success = (move_group.execute(initplan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
				if (success){
					std::cout<<"	Completed on planning iteration: "<<ii<<"/4\n";
					break;
				}
				ros::WallDuration(1.0).sleep();
			}

			ros::WallDuration(1.0).sleep();
		}
	}
	ros::spin();
	ros::WallDuration(1.0).sleep();
	return 0;
}
