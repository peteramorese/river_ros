#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "river_ros/PlanExecute.h"

bool eef_status(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
	if (request.data) {
		std::cout<<"yay im true"<<std::endl;
		response.success = true;
		response.message = "good job spookyboi";
	} else if (!request.data) {
		std::cout<<"boo im false"<<std::endl;
		response.success = false;
		response.message = "bad job try again";
	}
	return true;
}

bool planex_service(river_ros::PlanExecute::Request &request, river_ros::PlanExecute::Response &response) {
	if (request.plan_and_execute) {
		std::cout<<"im planning and executing"<<std::endl;
		response.status = "success";
	} else if (!request.plan_and_execute) {
		std::cout<<"i should not plan and execute"<<std::endl;
		response.status = "waiting";
	}
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "eef_srv_test_node");
	ros::NodeHandle eef_test_NH;
	ros::ServiceServer service1 = eef_test_NH.advertiseService("end_effector_status",eef_status);
	ros::ServiceServer service2 = eef_test_NH.advertiseService("planex_service",planex_service);
	std::cout<<"ready BOI?"<<std::endl;
	ros::spin();
	return 0;
}
