#include "ros/ros.h"
#include "std_srvs/SetBool.h"

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

int main(int argc, char **argv) {
	ros::init(argc, argv, "eef_srv_test_node");
	ros::NodeHandle eef_test_NH;
	ros::ServiceServer service = eef_test_NH.advertiseService("end_effector_status",eef_status);
	std::cout<<"ready BOI?"<<std::endl;
	ros::spin();
	return 0;
}
