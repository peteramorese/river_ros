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


class PlanningQuerySrv {
	private:
	public:
		bool planningQuery_serviceCB(geometry_msgs::PoseStamped::
	
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "manipulator_node");
	ros::NodeHandle node_handle("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();


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

	/* Define Table, Collision Environment, End Effector */ 
	// Table:
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

	// End Effector
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

	return 0;
}
