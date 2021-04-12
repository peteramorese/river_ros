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

// int count_max = 3;
// int count = 0;

short int Arr[4];

void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Pixy_Pack");

	ros::NodeHandle n;

	ros::Subscriber sub3 = n.subscribe("Pixy_Data", 100, arrayCallback);

	ros::Publisher chat_pub = n.advertise<river_ros::data_pkg>("arduino/data_read", 1000);

	river_ros::data_pkg pub_pkg; // Initialize the first publish package
	pub_pkg.msg_id = 0;

	river_ros::data_pt point;
	river_ros::data_pkt packet;
	river_ros::data_pkg package;
	package.msg_id = 1;

	int sid_prev = -1;
	int sid_curr;

	std::vector<int> sid_curr_msg;

	while (ros::ok())
	{

		chat_pub.publish(pub_pkg); // Publish the stored package
		ros::spinOnce(); // Get the next data point

		for(int j = 0; j < 4; j++)
		{
			printf("%d, ", Arr[j]);
		}
		printf("\n");

		point.x = Arr[0];
		point.y = Arr[1];
		point.mid = Arr[2];
		point.sid = Arr[3];

		sid_curr = point.sid;

		if(sid_prev == -1) // First data point of the message
		{
			sid_prev = sid_curr; // Set the previous sid to the current sid
			// sid_curr_msg.push_back(sid_curr); // Add the current sid to the list of sids in this message
		}

		if(sid_curr == sid_prev) // If the current point's sid is the same as the last point's sid
		{
			packet.pt.push_back(point); // Add this point to the same packet
		}
		else // If the current point's sid is different from the last point's sid
		{
			package.pkt.push_back(packet); // Add the packet to the package
			sid_curr_msg.push_back(sid_prev); // Add the previous point's sid to the list of sids in the package
			packet.pt.clear(); // Clear the packet
			// sid_prev = sid_curr; // Set the previous sid to the sid that just got added to the package

			// Search the list of sids in the message for the current sid, if found
			if(std::find(sid_curr_msg.begin(), sid_curr_msg.end(), sid_curr) != sid_curr_msg.end())
			{
				pub_pkg = package; // Place the current package in the slot to publish
				package.pkt.clear(); // Clear the package
				sid_curr_msg.clear(); // Clear the sids in the current msg
				package.msg_id = package.msg_id + 1; // Increment the msg ID

				
				// sid_prev = sid_curr;
			}

			packet.pt.push_back(point); // Add this point to the packet
			sid_prev = sid_curr; // Update the previous sid to this sid
		}

		// packet.pt.push_back(point);
		// count++;

	// 	if(count == count_max) {
	// 		count = 0;
	// 		package.pkt.push_back(packet);
	// 		packet.pt.clear();
	// 		chat_pub.publish(package);
	// 		ros::spinOnce();
	// //		loop_rate.sleep();
	// 		std::cout<<"msg published -------"<<std::endl;
	// 		//ROS_INFO("msg published");
	// 		package.pkt.clear();
	// 	}
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
