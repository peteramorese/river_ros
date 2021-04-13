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
	ros::Rate loop_rate(500);

	ros::Subscriber sub3 = n.subscribe("Pixy_Data", 100, arrayCallback);

	ros::Publisher chat_pub = n.advertise<river_ros::data_pkg>("arduino/data_read", 1000);

	river_ros::data_pkg pub_pkg; // Initialize the first publish package
	pub_pkg.stamp = ros::Time::now();

	river_ros::data_pt point;
	river_ros::data_pkt packet;
	river_ros::data_pkg package;

	int sid_prev = -1;
	int sid_curr;

	std::vector<int> count;

	std::vector<int> sid_curr_msg;

	while (ros::ok())
	{
		count.push_back(1);
		count.push_back(1);

		loop_rate.sleep();
		chat_pub.publish(pub_pkg); // Publish the stored package
		ros::spinOnce(); // Get the next data point

		if(true)
		{
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
					package.stamp = ros::Time::now();
					pub_pkg = package; // Place the current package in the slot to publish
					package.pkt.clear(); // Clear the package
					sid_curr_msg.clear(); // Clear the sids in the current msg

					
					// sid_prev = sid_curr;
				}

				packet.pt.push_back(point); // Add this point to the packet
				sid_prev = sid_curr; // Update the previous sid to this sid
			}
		}
		else
		{
			point.x = 5;
			point.y = 10;
			point.mid = 5;
			point.sid = 0;

			packet.pt.push_back(point);

			if(count[0] == 4)
			{
				package.pkt.push_back(packet);
				count[1]++;

				if(count[1] == 4)
				{
					package.stamp = ros::Time::now();
					pub_pkg = package;
					package.pkt.clear();
					std::cout << "Package Published " << pub_pkg.stamp << std::endl;
					count[1] = 1;
				}

				packet.pt.clear();
				count[0] = 1;
			}
			else
			{
				count[0]++;
			}
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
