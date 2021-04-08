#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "river_ros/PlanningQuery_srv.h"
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
		moveit::planning_interface::MoveGroupInterface* move_group_ptr;
		moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_ptr;
	public:
		PlanningQuerySrv(moveit::planning_interface::MoveGroupInterface* move_group_ptr_, moveit::planning_interface::PlanningSceneInterface* psi_ptr_) : 
			move_group_ptr(move_group_ptr_),
			planning_scene_interface_ptr(psi_ptr_)	{
			}
		const double bag_l = .4127;
		const double bag_w = .2286;
		const double bag_h = .2413;
		bool planQuery_serviceCB(river_ros::PlanningQuery_srv::Request &request, river_ros::PlanningQuery_srv::Response &response) {
			std::cout<<"\n";
			std::cout<<"HELLO"<<std::endl;
			std::cout<<"\n";
			ROS_INFO_NAMED("manipulator_node", "Recieved Planning Query");

			if (request.setup_environment) {
				std::vector<moveit_msgs::CollisionObject> col_obj_vec;
				col_obj_vec.resize(request.bag_poses.poses.size());

				for (int i=0; i<request.bag_poses.poses.size(); ++i) {
					col_obj_vec[i].header.frame_id = "base_link";
					col_obj_vec[i].id = request.bag_labels[i];
					col_obj_vec[i].primitives.resize(1);
					col_obj_vec[i].primitives[0].type = col_obj_vec[i].primitives[0].BOX;
					col_obj_vec[i].primitives[0].dimensions.resize(3);
					col_obj_vec[i].primitives[0].dimensions[0] = bag_l;
					col_obj_vec[i].primitives[0].dimensions[1] = bag_w;
					col_obj_vec[i].primitives[0].dimensions[2] = bag_h;

					col_obj_vec[i].primitive_poses.resize(1);
					std::cout<<" i see : "<<request.bag_poses.poses[i].position.x<<std::endl;
					std::cout<<" i see : "<<request.bag_poses.poses[i].position.y<<std::endl;
					std::cout<<" i see : "<<request.bag_poses.poses[i].position.z<<std::endl;
					col_obj_vec[i].primitive_poses[0].position.x = request.bag_poses.poses[i].position.x;
					col_obj_vec[i].primitive_poses[0].position.y = request.bag_poses.poses[i].position.y;
					col_obj_vec[i].primitive_poses[0].position.z = request.bag_poses.poses[i].position.z;
					col_obj_vec[i].primitive_poses[0].orientation.x = request.bag_poses.poses[i].orientation.x;
					col_obj_vec[i].primitive_poses[0].orientation.y = request.bag_poses.poses[i].orientation.y;
					col_obj_vec[i].primitive_poses[0].orientation.z = request.bag_poses.poses[i].orientation.z;
					col_obj_vec[i].primitive_poses[0].orientation.w = request.bag_poses.poses[i].orientation.w;
					col_obj_vec[i].operation = col_obj_vec[i].ADD;
				}
				planning_scene_interface_ptr->applyCollisionObjects(col_obj_vec);
				planning_scene_interface_ptr->addCollisionObjects(col_obj_vec);
				std::cout<<"done setting up env"<<std::endl;
			}

			if (request.pickup_object != "none") {
				std::cout<<"attaching obj"<<std::endl;
				std::string obj_label = request.pickup_object;
				move_group_ptr->attachObject(obj_label,"ee_link");
				response.success = true;
				std::cout<<"done attaching obj"<<std::endl;
			} else if (request.drop_object != "none") {
				std::cout<<"done droppin obj"<<std::endl;
				std::string obj_label = request.drop_object;
				move_group_ptr->detachObject(obj_label);
				response.success = true;
				std::cout<<"done dropping obj"<<std::endl;
			} else {
				std::cout<<"moving"<<std::endl;
				geometry_msgs::Pose pose;
				moveit::planning_interface::MoveGroupInterface::Plan plan;

				move_group_ptr->setStartStateToCurrentState();

				// Convert from PoseStamed to Pose
				pose.position.x = request.manipulator_pose.position.x;
				pose.position.y = request.manipulator_pose.position.y;
				pose.position.z = request.manipulator_pose.position.z + .1 + bag_h/2;
				pose.orientation.x = 0;//-request.manipulator_pose.orientation.x;
				pose.orientation.y = 0;//-request.manipulator_pose.orientation.y;
				pose.orientation.z = 0;//-request.manipulator_pose.orientation.z;
				pose.orientation.w = 1;//request.manipulator_pose.orientation.w;
				move_group_ptr->setPoseTarget(pose);
				move_group_ptr->setPlanningTime(5.0);

				ROS_INFO_NAMED("manipulator_node", "Reference frame: %s", move_group_ptr->getPlanningFrame().c_str());
				std::cout<<"moving to x: "<< request.manipulator_pose.position.x<<std::endl;
				std::cout<<"moving to y: "<< request.manipulator_pose.position.y<<std::endl;
				std::cout<<"moving to z: "<< request.manipulator_pose.position.z<<std::endl;
				bool success = false;
				for (int ii=0; ii<4; ii++){
					std::cout<<"before plan"<<std::endl;
					//move_group_ptr->plan(plan);
					std::cout<<"before execute"<<std::endl;
					//success = (move_group_ptr->execute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
					success = (move_group_ptr->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);
					if (success){
						ROS_INFO_NAMED("manipulator_node","Completed planning on iteration: %d",ii);
						break;
					}
					ros::WallDuration(1.0).sleep();
				}
				response.success = success;
				std::cout<<"done moving"<<std::endl;
			}
			return true;
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
	colObjVec[0].primitive_poses[0].position.z = -1; // should be -.5
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

	PlanningQuerySrv plan_query_srv_container(&move_group, &planning_scene_interface);

	ros::ServiceServer plan_query_service = M_NH.advertiseService("task_planner/planning_query", &PlanningQuerySrv::planQuery_serviceCB, &plan_query_srv_container);

	ros::waitForShutdown();
	return 0;
}
