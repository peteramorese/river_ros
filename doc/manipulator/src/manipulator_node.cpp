#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "river_ros/PlanningQuery_msg.h"
#include "river_ros/PlanningQueryStatus_msg.h"
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


class PlanningQueryExecute {
	private:
		std::string& action;
	public:
		std::vector<moveit_msgs::CollisionObject> col_obj_vec;
		geometry_msgs::PoseStamped arm_pose;
		std::string pickup_obj;
		std::string drop_obj;
		PlanningQueryExecute(std::string& action_) : action(action_) {
			action = "none";
		}
		void planQueryExecuteCB(const river_ros::PlanningQuery_msg::ConstPtr& plan_query) {
			action = plan_query->action;
			std::cout<<"\n";
			std::cout<<"HELLO"<<std::endl;
			std::cout<<"\n";
			ROS_INFO_NAMED("manipulator_node", "Recieved Planning Query");

			if (plan_query->setup_environment) {
				col_obj_vec.resize(plan_query->bag_poses.poses.size());

				for (int i=0; i<plan_query->bag_poses.poses.size(); ++i) {
					col_obj_vec[i].header.frame_id = "base_link";
					std::string temp_str = plan_query->bag_labels[i];
					col_obj_vec[i].id = temp_str;
					col_obj_vec[i].primitives.resize(1);
					col_obj_vec[i].primitives[0].type = col_obj_vec[i].primitives[0].BOX;
					// Bag definition
					col_obj_vec[i].primitives[0].dimensions.resize(3);
					col_obj_vec[i].primitives[0].dimensions[0] = .4127;
					col_obj_vec[i].primitives[0].dimensions[1] = .2286;
					col_obj_vec[i].primitives[0].dimensions[2] = .2413;

					col_obj_vec[i].primitive_poses.resize(1);
					std::cout<<" i see : "<<plan_query->bag_poses.poses[i].position.x<<std::endl;
					std::cout<<" i see : "<<plan_query->bag_poses.poses[i].position.y<<std::endl;
					std::cout<<" i see : "<<plan_query->bag_poses.poses[i].position.z<<std::endl;
					col_obj_vec[i].primitive_poses[0].position.x = plan_query->bag_poses.poses[i].position.x;
					col_obj_vec[i].primitive_poses[0].position.y = plan_query->bag_poses.poses[i].position.y;
					col_obj_vec[i].primitive_poses[0].position.z = plan_query->bag_poses.poses[i].position.z;
					col_obj_vec[i].primitive_poses[0].orientation.x = plan_query->bag_poses.poses[i].orientation.x;
					col_obj_vec[i].primitive_poses[0].orientation.y = plan_query->bag_poses.poses[i].orientation.y;
					col_obj_vec[i].primitive_poses[0].orientation.z = plan_query->bag_poses.poses[i].orientation.z;
					col_obj_vec[i].primitive_poses[0].orientation.w = plan_query->bag_poses.poses[i].orientation.w;
					col_obj_vec[i].operation = col_obj_vec[i].ADD;
				}
			}
			arm_pose.pose.position.x = plan_query->manipulator_pose.pose.position.x;
			arm_pose.pose.position.y = plan_query->manipulator_pose.pose.position.y;
			arm_pose.pose.position.z = plan_query->manipulator_pose.pose.position.z;
			arm_pose.pose.orientation.x = plan_query->manipulator_pose.pose.orientation.x;
			arm_pose.pose.orientation.y = plan_query->manipulator_pose.pose.orientation.y;
			arm_pose.pose.orientation.z = plan_query->manipulator_pose.pose.orientation.z;
			arm_pose.pose.orientation.w = plan_query->manipulator_pose.pose.orientation.w;
			pickup_obj = plan_query->pickup_object;
			drop_obj = plan_query->drop_object;


		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "manipulator_node");
	ros::NodeHandle M_NH("~");
	
	ros::AsyncSpinner spinner(2);
	spinner.start();


	/////////////////////////////////////////////////////////////////////////
	static const std::string PLANNING_GROUP = "manipulator";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
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

	if (!M_NH.getParam("planning_plugin", planner_plugin_name))
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
		if (!planner_instance->initialize(robot_model, M_NH.getNamespace()))
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

	ROS_INFO_NAMED("manipulator_node", "Reference frame: %s", move_group.getPlanningFrame().c_str());

	// Subscribe to the planning query
	std::string action_PQE;
	PlanningQueryExecute ex_PQ_sub_container(action_PQE);
	ros::Subscriber plan_query_execute_sub = M_NH.subscribe("task_planner/planning_query", 1, &PlanningQueryExecute::planQueryExecuteCB, &ex_PQ_sub_container);

	// Publish action and success to planning query
	river_ros::PlanningQueryStatus_msg plan_query_status_msg;
	plan_query_status_msg.action = "idle";
	ros::Publisher plan_query_pub = M_NH.advertise<river_ros::PlanningQueryStatus_msg>("planning_query_ex_status", 1);
	plan_query_pub.publish(plan_query_status_msg);

	int Nobj=2;
	//int Nobj=1;
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
	colObjVec[0].primitive_poses[0].position.z = -.6; // should be -.5
	colObjVec[0].primitive_poses[0].orientation.x = 0;
	colObjVec[0].primitive_poses[0].orientation.y = 0;
	colObjVec[0].primitive_poses[0].orientation.z = 0;
	colObjVec[0].primitive_poses[0].orientation.w = 1;
	colObjVec[0].operation = colObjVec[0].ADD;

	// End Effector
	colObjVec[1].id = "eef";
	colObjVec[1].primitives.resize(1);
	colObjVec[1].primitives[0].type = colObjVec[1].primitives[0].BOX;
	colObjVec[1].primitives[0].dimensions.resize(3);
	colObjVec[1].primitives[0].dimensions[0] = .09;
	colObjVec[1].primitives[0].dimensions[1] = .1;
	colObjVec[1].primitives[0].dimensions[2] = .12;

	colObjVec[1].primitive_poses.resize(1);
	colObjVec[1].primitive_poses[0].position.x =.09*.5+.01;
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

	ros::Rate r(3);
	while (ros::ok()) {
		r.sleep();
		plan_query_status_msg.success = "in progress";
		plan_query_pub.publish(plan_query_status_msg);
		std::cout<<"action_PQE entering: "<<action_PQE<<std::endl;
		if (action_PQE == "begin") {
			std::cout<<"published working ... "<<std::endl;
			plan_query_status_msg.action = "working";
			plan_query_pub.publish(plan_query_status_msg);
			planning_scene_interface.applyCollisionObjects(ex_PQ_sub_container.col_obj_vec);
			planning_scene_interface.addCollisionObjects(ex_PQ_sub_container.col_obj_vec);
			std::cout<<"done setting up env"<<std::endl;
			if (ex_PQ_sub_container.pickup_obj != "none") {
				std::cout<<"attaching obj"<<std::endl;
				move_group.attachObject(ex_PQ_sub_container.pickup_obj,"ee_link");
				plan_query_status_msg.success = "success";
				plan_query_pub.publish(plan_query_status_msg);
				std::cout<<"done attaching obj"<<std::endl;
			} else if (ex_PQ_sub_container.drop_obj != "none") {
				std::cout<<"done droppin obj"<<std::endl;
				move_group.detachObject(ex_PQ_sub_container.drop_obj);
				plan_query_status_msg.success = "success";
				plan_query_pub.publish(plan_query_status_msg);
				std::cout<<"done dropping obj"<<std::endl;
			} else {
				std::cout<<"moving"<<std::endl;
				geometry_msgs::Pose pose;
				moveit::planning_interface::MoveGroupInterface::Plan plan;

				// Convert from PoseStamped to Pose
				pose.position.x = ex_PQ_sub_container.arm_pose.pose.position.x;
				pose.position.y = ex_PQ_sub_container.arm_pose.pose.position.y;
				pose.position.z = ex_PQ_sub_container.arm_pose.pose.position.z;
				pose.orientation.x = ex_PQ_sub_container.arm_pose.pose.orientation.x;
				pose.orientation.y = ex_PQ_sub_container.arm_pose.pose.orientation.y;
				pose.orientation.z = ex_PQ_sub_container.arm_pose.pose.orientation.z;
				pose.orientation.w = ex_PQ_sub_container.arm_pose.pose.orientation.w;

				move_group.setStartStateToCurrentState();
				move_group.setPoseTarget(pose);
				move_group.setPlanningTime(5.0);

				ROS_INFO_NAMED("manipulator_node", "Reference frame: %s", move_group.getPlanningFrame().c_str());
				std::cout<<"moving to x: "<< ex_PQ_sub_container.arm_pose.pose.position.x<<std::endl;
				std::cout<<"moving to y: "<< ex_PQ_sub_container.arm_pose.pose.position.y<<std::endl;
				std::cout<<"moving to z: "<< ex_PQ_sub_container.arm_pose.pose.position.z<<std::endl;
				bool success;
				for (int ii=0; ii<4; ii++){
					std::cout<<"before plan"<<std::endl;
					//move_group_ptr->plan(plan);
					std::cout<<"before execute"<<std::endl;
					//success = (move_group_ptr->execute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
					success = (move_group.move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);
					if (success){
						ROS_INFO_NAMED("manipulator_node","Completed planning on iteration: %d",ii);
						break;
					}
					ros::WallDuration(1.0).sleep();
				}
				if (success) {
					plan_query_status_msg.success = "success";
					plan_query_pub.publish(plan_query_status_msg);
				} else {
					plan_query_status_msg.success = "failed";
					plan_query_pub.publish(plan_query_status_msg);
				}
				std::cout<<"done moving"<<std::endl;
			}
			ros::Rate r_PQE(10);
			while (ros::ok() && action_PQE != "idle") {
				std::cout<<"action_PQE: "<<action_PQE<<std::endl;
				r_PQE.sleep();
			}

		}

	}

	ros::waitForShutdown();
	return 0;
}
