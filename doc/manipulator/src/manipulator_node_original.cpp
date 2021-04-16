#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "river_ros/PlanningQuery_srv.h"
#include <vector>
#include <cmath>

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
		std::vector<moveit_msgs::CollisionObject> col_obj_vec;
		std::vector<std::string> obs_domain_labels;
	public:
		PlanningQuerySrv(moveit::planning_interface::MoveGroupInterface* move_group_ptr_, moveit::planning_interface::PlanningSceneInterface* psi_ptr_, int N_TRIALS_) : 
			move_group_ptr(move_group_ptr_),
			planning_scene_interface_ptr(psi_ptr_),
			N_TRIALS(N_TRIALS_) {
			}
		const double bag_l = .4127;
		const double bag_w = .2286;
		const double bag_h = .2413;
		const int N_TRIALS;
		void setWorkspace(std::vector<moveit_msgs::CollisionObject> col_obj_vec_ws, std::vector<std::string> col_obj_vec_dom_lbls) {
			col_obj_vec.clear();		
			obs_domain_labels.clear();
			col_obj_vec = col_obj_vec_ws;
			obs_domain_labels = col_obj_vec_dom_lbls;
		}
		void setupEnvironment(std::string planning_domain_lbl) {
			std::cout<<"recieved planning domain label in setupEnvironment: "<<planning_domain_lbl<<std::endl;
			std::cout<<"obs_domain_labels size: "<<obs_domain_labels.size()<<std::endl;
			std::vector<moveit_msgs::CollisionObject> temp_col_vec;
			for (int i=0; i<col_obj_vec.size(); ++i) {
				if (obs_domain_labels[i] == "none") {
					ROS_INFO_NAMED("manipulator_node", "Environment Setup: Ignoring object with label: %s", col_obj_vec[i].id.c_str());
				} else if (obs_domain_labels[i] == planning_domain_lbl) {
					std::cout<<"adding:"<< col_obj_vec[i].id<<std::endl;
					col_obj_vec[i].operation = col_obj_vec[i].ADD;
					temp_col_vec.push_back(col_obj_vec[i]);
				} else {
					std::cout<<"removing:"<< col_obj_vec[i].id<<std::endl;
					col_obj_vec[i].operation = col_obj_vec[i].REMOVE;
					temp_col_vec.push_back(col_obj_vec[i]);
					std::cout<<"made it out of removing!"<<std::endl;
				}
			}
			std::cout<<"made it out of loop"<<std::endl;
			planning_scene_interface_ptr->applyCollisionObjects(temp_col_vec);
			//planning_scene_interface_ptr->addCollisionObjects(col_obj_vec);

		}
		void findObjAndUpdate(std::string obj_id, std::string domain_label_) {
			bool not_found = true;
			for (int i=0; i<col_obj_vec.size(); ++i) {
				if (col_obj_vec[i].id == obj_id) {
					std::cout<<"FIND UPDATE: found object id: "<<obj_id<<std::endl;
					std::cout<<"FIND UPDATE: updating to domain label: "<<domain_label_<<std::endl;
					obs_domain_labels[i] = domain_label_;	
					not_found = false;
				}	
			}
			if (not_found) {
				ROS_ERROR_NAMED("manipulator_node","Object id was not found. Cannot update domain label");
			}
		}
		bool planQuery_serviceCB(river_ros::PlanningQuery_srv::Request &request, river_ros::PlanningQuery_srv::Response &response) {
			std::cout<<"\n";
			std::cout<<"HELLO"<<std::endl;
			std::cout<<"Recieved planning domain: "<<request.planning_domain<<std::endl;
			std::cout<<"\n";
			ROS_INFO_NAMED("manipulator_node", "Recieved Planning Query");

			if (request.setup_environment) {
				//col_obj_vec.resize(request.bag_poses.poses.size());
				//bag_domain_labels = request.bag_domain_labels;

				for (int i=0; i<request.bag_poses.poses.size(); ++i) {
					moveit_msgs::CollisionObject temp_col_obj;
					temp_col_obj.header.frame_id = "world";
					temp_col_obj.id = request.bag_labels[i];
					temp_col_obj.primitives.resize(1);
					temp_col_obj.primitives[0].type = col_obj_vec[i].primitives[0].BOX;
					temp_col_obj.primitives[0].dimensions.resize(3);
					temp_col_obj.primitives[0].dimensions[0] = bag_l;
					temp_col_obj.primitives[0].dimensions[1] = bag_w;
					temp_col_obj.primitives[0].dimensions[2] = bag_h;

					temp_col_obj.primitive_poses.resize(1);
					std::cout<<" i see : "<<request.bag_poses.poses[i].position.x<<std::endl;
					std::cout<<" i see : "<<request.bag_poses.poses[i].position.y<<std::endl;
					std::cout<<" i see : "<<request.bag_poses.poses[i].position.z<<std::endl;
					temp_col_obj.primitive_poses[0].position.x = request.bag_poses.poses[i].position.x;
					temp_col_obj.primitive_poses[0].position.y = request.bag_poses.poses[i].position.y;
					temp_col_obj.primitive_poses[0].position.z = request.bag_poses.poses[i].position.z;
					temp_col_obj.primitive_poses[0].orientation.x = request.bag_poses.poses[i].orientation.x;
					temp_col_obj.primitive_poses[0].orientation.y = request.bag_poses.poses[i].orientation.y;
					temp_col_obj.primitive_poses[0].orientation.z = request.bag_poses.poses[i].orientation.z;
					temp_col_obj.primitive_poses[0].orientation.w = request.bag_poses.poses[i].orientation.w;
					col_obj_vec.push_back(temp_col_obj);
					obs_domain_labels.push_back(request.bag_domain_labels[i]);
					std::cout<<"Adding object: "<<temp_col_obj.id<<" to domain: "<<request.bag_domain_labels[i]<<std::endl;
					//col_obj_vec[i].operation = col_obj_vec[i].ADD;
				}
				//planning_scene_interface_ptr->applyCollisionObjects(col_obj_vec);
				//planning_scene_interface_ptr->addCollisionObjects(col_obj_vec);
				std::cout<<"done setting up env"<<std::endl;
				setupEnvironment(request.planning_domain);
			}

			if (request.pickup_object != "none") {
				std::cout<<"attaching obj"<<std::endl;
				std::string obj_label = request.pickup_object;
				move_group_ptr->attachObject(obj_label,"ee_link");
				// Change the domain of the attached object to be 'none' 
				// so that it does not get removed
				findObjAndUpdate(obj_label, "none");
				response.success = true;
				ROS_INFO_NAMED("manipulator_node","Done attaching object");
			} else if (request.drop_object != "none") {
				std::cout<<"done droppin obj"<<std::endl;
				std::string obj_label = request.drop_object;
				move_group_ptr->detachObject(obj_label);
				// If we are releasing an object, the new domain becomes
				// whatever domain the end effector is in (the request)
				findObjAndUpdate(obj_label, request.planning_domain);
				response.success = true;
				ROS_INFO_NAMED("manipulator_node","Done detaching object");
			} else {
				setupEnvironment(request.planning_domain);
				std::cout<<"moving"<<std::endl;
				geometry_msgs::Pose pose;
				moveit::planning_interface::MoveGroupInterface::Plan plan;

				move_group_ptr->setStartStateToCurrentState();

				// Convert from PoseStamed to Pose
				if (request.safe_config) {
					std::vector<double> joint_val_target = {0.0,-2.094,1.763,-1.222,-1.641,0.0};
					move_group_ptr->setJointValueTarget(joint_val_target);
				} else {
					tf2::Quaternion q_orig, q_in, q_f, q_rot, q_90;
					q_orig[0] = 0;
					q_orig[1] = 0;
					q_orig[2] = bag_h/2 + .9;
					q_orig[3] = 0;

					tf2::convert(request.manipulator_pose.orientation, q_in);
					q_90.setRPY(0, M_PI/2, 0);
					q_f = q_in * q_orig * q_in.inverse(); 
					q_rot = q_in * q_90;

					pose.position.x = request.manipulator_pose.position.x + q_f[0];
					pose.position.y = request.manipulator_pose.position.y + q_f[1];
					pose.position.z = request.manipulator_pose.position.z + q_f[2];
					pose.orientation.x = q_rot[0];
					pose.orientation.y = q_rot[1];  
					pose.orientation.z = q_rot[2];
					pose.orientation.w = q_rot[3];
					move_group_ptr->setPoseTarget(pose);
				}
				move_group_ptr->setPlanningTime(15.0);

				ROS_INFO_NAMED("manipulator_node", "Reference frame: %s", move_group_ptr->getPlanningFrame().c_str());
				std::cout<<"moving to x: "<< request.manipulator_pose.position.x<<std::endl;
				std::cout<<"moving to y: "<< request.manipulator_pose.position.y<<std::endl;
				std::cout<<"moving to z: "<< request.manipulator_pose.position.z<<std::endl;
				bool success = false;
				for (int ii=0; ii<N_TRIALS; ii++){
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

	//int Nobj=2;
	//int Nobj=1;
	std::vector<moveit_msgs::CollisionObject> colObjVec;
	std::vector<std::string> colObjVec_domain_lbls;
	//colObjVec.resize(Nobj);
	//colObjVec_domain_lbls.resize(Nobj);
	int ind = 0;
	/*
	   for (int i=0; i<Nobj; i++) {
	   colObjVec[i].header.frame_id = "base_link";
	   }
	   */


	/* Define Table, Collision Environment, End Effector */ 
	
	// ground:
	/*
	moveit_msgs::CollisionObject ground;
	ground.header.frame_id = "world";
	ground.id = "ground_p";
	ground.primitives.resize(1);
	ground.primitives[0].type = colObjVec[0].primitives[0].BOX;
	ground.primitives[0].dimensions.resize(3);
	ground.primitives[0].dimensions[0] = 2;
	ground.primitives[0].dimensions[1] = 2.4;
	ground.primitives[0].dimensions[2] = .1;

	ground.primitive_poses.resize(1);
	ground.primitive_poses[0].position.x = 0;
	ground.primitive_poses[0].position.y = 0;
	ground.primitive_poses[0].position.z = -.05; // should be -.5
	ground.primitive_poses[0].orientation.x = 0;
	ground.primitive_poses[0].orientation.y = 0;
	ground.primitive_poses[0].orientation.z = 0;
	ground.primitive_poses[0].orientation.w = 1;
	ground.operation = ground.ADD;
	colObjVec.push_back(ground);
	colObjVec_domain_lbls.push_back("pickup domain");

	ground.id = "ground_d";
	colObjVec.push_back(ground);
	colObjVec_domain_lbls.push_back("dropoff domain");

	// ceiling:
	moveit_msgs::CollisionObject ceiling;
	ceiling.header.frame_id = "world";
	ceiling.id = "ceiling_p";
	ceiling.primitives.resize(1);
	ceiling.primitives[0].type = colObjVec[0].primitives[0].BOX;
	ceiling.primitives[0].dimensions.resize(3);
	ceiling.primitives[0].dimensions[0] = 2;
	ceiling.primitives[0].dimensions[1] = 2.4;
	ceiling.primitives[0].dimensions[2] = .1;

	ceiling.primitive_poses.resize(1);
	ceiling.primitive_poses[0].position.x = 0;
	ceiling.primitive_poses[0].position.y = 0;
	ceiling.primitive_poses[0].position.z = 1.8+.1/2; // should be -.5
	ceiling.primitive_poses[0].orientation.x = 0;
	ceiling.primitive_poses[0].orientation.y = 0;
	ceiling.primitive_poses[0].orientation.z = 0;
	ceiling.primitive_poses[0].orientation.w = 1;
	ceiling.operation = ceiling.ADD;
	colObjVec.push_back(ceiling);
	colObjVec_domain_lbls.push_back("pickup domain");

	ceiling.id = "ceiling_d";
	colObjVec.push_back(ceiling);
	colObjVec_domain_lbls.push_back("dropoff domain");
	*/


	// table:
	/*
	moveit_msgs::CollisionObject table;
	table.header.frame_id = "world";
	table.id = "table";
	table.primitives.resize(1);
	table.primitives[0].type = colObjVec[0].primitives[0].BOX;
	table.primitives[0].dimensions.resize(3);
	table.primitives[0].dimensions[0] = .5;
	table.primitives[0].dimensions[1] = 1.1;
	table.primitives[0].dimensions[2] = .56;

	table.primitive_poses.resize(1);
	table.primitive_poses[0].position.x = -.34-.5/2; //.34
	table.primitive_poses[0].position.y = 0;
	table.primitive_poses[0].position.z = .56/2; // should be -.5
	table.primitive_poses[0].orientation.x = 0;
	table.primitive_poses[0].orientation.y = 0;
	table.primitive_poses[0].orientation.z = 0;
	table.primitive_poses[0].orientation.w = 1;
	table.operation = table.ADD;
	colObjVec.push_back(table);
	colObjVec_domain_lbls.push_back("pickup domain");
	*/

	// drop off table
	/*
	moveit_msgs::CollisionObject table_dropoff;
	table_dropoff.header.frame_id = "world";
	table_dropoff.id = "table_dropoff";
	table_dropoff.primitives.resize(1);
	table_dropoff.primitives[0].type = colObjVec[0].primitives[0].BOX;
	table_dropoff.primitives[0].dimensions.resize(3);
	table_dropoff.primitives[0].dimensions[0] = .5;
	table_dropoff.primitives[0].dimensions[1] = 1.1;
	table_dropoff.primitives[0].dimensions[2] = .939;

	table_dropoff.primitive_poses.resize(1);
	table_dropoff.primitive_poses[0].position.x = .534 + .5/2;
	table_dropoff.primitive_poses[0].position.y = 0;
	table_dropoff.primitive_poses[0].position.z = .939/2; // should be -.5
	table_dropoff.primitive_poses[0].orientation.x = 0;
	table_dropoff.primitive_poses[0].orientation.y = 0;
	table_dropoff.primitive_poses[0].orientation.z = 0;
	table_dropoff.primitive_poses[0].orientation.w = 1;
	table_dropoff.operation = table_dropoff.ADD;
	colObjVec.push_back(table_dropoff);
	colObjVec_domain_lbls.push_back("dropoff domain");

	// UR box and motor 
	moveit_msgs::CollisionObject ur_box;
	ur_box.header.frame_id = "world";
	ur_box.id = "ur_box";
	ur_box.primitives.resize(1);
	ur_box.primitives[0].type = colObjVec[0].primitives[0].BOX;
	ur_box.primitives[0].dimensions.resize(3);
	ur_box.primitives[0].dimensions[0] = .5;
	ur_box.primitives[0].dimensions[1] = 1.1;
	ur_box.primitives[0].dimensions[2] = .2032;

	ur_box.primitive_poses.resize(1);
	ur_box.primitive_poses[0].position.x = .254 + (.534-.254)/2;
	ur_box.primitive_poses[0].position.y = 0;
	ur_box.primitive_poses[0].position.z = .2032/2; // should be -.5
	ur_box.primitive_poses[0].orientation.x = 0;
	ur_box.primitive_poses[0].orientation.y = 0;
	ur_box.primitive_poses[0].orientation.z = 0;
	ur_box.primitive_poses[0].orientation.w = 1;
	ur_box.operation = ur_box.ADD;
	colObjVec.push_back(ur_box);
	colObjVec_domain_lbls.push_back("dropoff domain");


	// pressure regulator:
	moveit_msgs::CollisionObject pres_reg;
	pres_reg.header.frame_id = "world";
	pres_reg.id = "pres_reg";
	pres_reg.primitives.resize(1);
	pres_reg.primitives[0].type = colObjVec[0].primitives[0].BOX;
	pres_reg.primitives[0].dimensions.resize(3);
	pres_reg.primitives[0].dimensions[0] = .1;
	pres_reg.primitives[0].dimensions[1] = .3;
	pres_reg.primitives[0].dimensions[2] = .14;

	pres_reg.primitive_poses.resize(1);
	pres_reg.primitive_poses[0].position.x = -.19-.1/2; //.34
	pres_reg.primitive_poses[0].position.y = 0;
	pres_reg.primitive_poses[0].position.z = .14/2; // should be -.5
	pres_reg.primitive_poses[0].orientation.x = 0;
	pres_reg.primitive_poses[0].orientation.y = 0;
	pres_reg.primitive_poses[0].orientation.z = 0;
	pres_reg.primitive_poses[0].orientation.w = 1;
	pres_reg.operation = pres_reg.ADD;
	colObjVec.push_back(pres_reg);
	colObjVec_domain_lbls.push_back("pickup domain");

	
	// left wall:
	moveit_msgs::CollisionObject wall_L;
	wall_L.header.frame_id = "world";
	wall_L.id = "wall_L_p";
	wall_L.primitives.resize(1);
	wall_L.primitives[0].type = colObjVec[0].primitives[0].BOX;
	wall_L.primitives[0].dimensions.resize(3);
	wall_L.primitives[0].dimensions[0] = 2;
	wall_L.primitives[0].dimensions[1] = .1;
	wall_L.primitives[0].dimensions[2] = 1.8;

	wall_L.primitive_poses.resize(1);
	wall_L.primitive_poses[0].position.x = 0; //.34
	wall_L.primitive_poses[0].position.y = -1.15;
	wall_L.primitive_poses[0].position.z = .9; // should be -.5
	wall_L.primitive_poses[0].orientation.x = 0;
	wall_L.primitive_poses[0].orientation.y = 0;
	wall_L.primitive_poses[0].orientation.z = 0;
	wall_L.primitive_poses[0].orientation.w = 1;
	wall_L.operation = wall_L.ADD;
	colObjVec.push_back(wall_L);
	colObjVec_domain_lbls.push_back("pickup domain");

	wall_L.id = "wall_L_d";
	colObjVec.push_back(wall_L);
	colObjVec_domain_lbls.push_back("dropoff domain");

	// right wall:
	moveit_msgs::CollisionObject wall_R;
	wall_R.header.frame_id = "world";
	wall_R.id = "wall_R_p";
	wall_R.primitives.resize(1);
	wall_R.primitives[0].type = colObjVec[0].primitives[0].BOX;
	wall_R.primitives[0].dimensions.resize(3);
	wall_R.primitives[0].dimensions[0] = 2;
	wall_R.primitives[0].dimensions[1] = .1;
	wall_R.primitives[0].dimensions[2] = 1.8;

	wall_R.primitive_poses.resize(1);
	wall_R.primitive_poses[0].position.x = 0; //.34
	wall_R.primitive_poses[0].position.y = 1.15;
	wall_R.primitive_poses[0].position.z = .9; // should be -.5
	wall_R.primitive_poses[0].orientation.x = 0;
	wall_R.primitive_poses[0].orientation.y = 0;
	wall_R.primitive_poses[0].orientation.z = 0;
	wall_R.primitive_poses[0].orientation.w = 1;
	wall_R.operation = wall_R.ADD;
	colObjVec.push_back(wall_R);
	colObjVec_domain_lbls.push_back("pickup domain");

	wall_R.id = "wall_R_d";
	colObjVec.push_back(wall_R);
	colObjVec_domain_lbls.push_back("dropoff domain");
	*/

	// End Effector
	moveit_msgs::CollisionObject eef;
	eef.header.frame_id = "ee_link";
	eef.id = "eef";
	eef.primitives.resize(1);
	eef.primitives[0].type = colObjVec[1].primitives[0].BOX;
	eef.primitives[0].dimensions.resize(3);
	eef.primitives[0].dimensions[0] = .09;
	eef.primitives[0].dimensions[1] = .1;
	eef.primitives[0].dimensions[2] = .12;

	eef.primitive_poses.resize(1);
	eef.primitive_poses[0].position.x =.09*.5;
	eef.primitive_poses[0].position.y = 0;
	eef.primitive_poses[0].position.z = .02; 
	eef.primitive_poses[0].orientation.x = 0;
	eef.primitive_poses[0].orientation.y = 0;
	eef.primitive_poses[0].orientation.z = 0;
	eef.primitive_poses[0].orientation.w = 1;
	eef.operation = eef.ADD;
	colObjVec.push_back(eef);
	colObjVec_domain_lbls.push_back("none");

	planning_scene_interface.applyCollisionObjects(colObjVec);
	//planning_scene_interface.addCollisionObjects(colObjVec);
	move_group.attachObject("eef","ee_link");
	move_group.setEndEffectorLink("ee_link");

	PlanningQuerySrv plan_query_srv_container(&move_group, &planning_scene_interface, 5);
	plan_query_srv_container.setWorkspace(colObjVec, colObjVec_domain_lbls);

	ros::ServiceServer plan_query_service = M_NH.advertiseService("task_planner/planning_query", &PlanningQuerySrv::planQuery_serviceCB, &plan_query_srv_container);

	ros::waitForShutdown();
	return 0;
}
