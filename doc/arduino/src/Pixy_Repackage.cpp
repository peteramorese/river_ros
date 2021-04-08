
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "river_ros/data_pt.h"
#include "river_ros/data_pkt.h"
#include "river_ros/data_pkg.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"

int count_max = 3;
int count = 0;
short int Arr[4];

river_ros::data_pt point;
river_ros::data_pkt packet;
river_ros::data_pkg package;

void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Pixy_Pack");

	ros::NodeHandle n;

	ros::Subscriber sub3 = n.subscribe("Pixy_Data", 100, arrayCallback);

	ros::Publisher chat_pub = n.advertise<river_ros::data_pkg>("chatter", 1000);

	while (ros::ok()) {


		for(int j = 0; j < 4; j++)
		{
			printf("%d, ", Arr[j]);
		}

		printf("\n");
	point.x = Arr[0];
	point.y = Arr[1];
	point.mid = Arr[2];
	point.sid = Arr[3];

	packet.pt.push_back(point);
	count++;

	if(count == count_max) {
		count = 0;
		package.pkt.push_back(packet);
		packet.pt.clear();
		chat_pub.publish(package);
		ros::spinOnce();
//		loop_rate.sleep();
		ROS_INFO("msg published");
		package.pkt.clear();
	}
	}
		return 0;

}
void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array)

{

	int i = 0;
	int count = 0;

	// print all the remaining numbers
	for(std::vector<short int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;

		i++;
	}


	return;
}
