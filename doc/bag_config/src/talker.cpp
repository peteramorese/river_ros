/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "river_ros/data_pt.h"
#include "river_ros/data_pkt.h"
#include "river_ros/data_pkg.h"
// %EndTag(MSG_HEADER)%

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <limits.h>
#include <unistd.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker_ros");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<river_ros::data_pkg>("chatter", 1000);

  ros::Rate loop_rate(5);

  std::string path = "src/river_ros/doc/bag_config/data/";
  int num_files = 5;
  // std::string filename[num_files] = {"cam0_final.txt", "cam1_final.txt", "cam2_final.txt"};
  std::string filename[num_files] = {"Y_cal0.txt", "Y_cal1.txt", "Y_cal2.txt", "Y_cal3.txt", "Y_cal4.txt"};
  std::vector<std::ifstream*> files;

  // std::ifstream file;
  // file.open(path + filename[0]);
  // files.push_back(&file);

  // for(int i = 0; i < num_files; i++)
  // {
  //   std::ifstream file;
  //   file.open(path + filename[i]);
  //   files.push_back(&file);
  //   // std::cout << "i = " << i << " open " << files[i]->good() << std::endl;
  // }


  std::string str;
  std::string substr;
  int cnt;
  int count = 0;
  int count_max = 3;
  int file_id = 0;
  int super_count = 0;
  river_ros::data_pt point;
  river_ros::data_pkt packet;
  river_ros::data_pkg package;

  // std::getline(*files[file_id], str);
  // std::cout << str << std::endl;
  // std::getline(*files[file_id+1], str);
  // std::cout << "'" << str << "'" << std::endl;


  // Array of ifstream objects only keeps files open with these four lines here??????
  std::ifstream file1;
  file1.open(path +filename[0]);
  std::ifstream file2;
  file2.open(path + filename[1]);
  std::ifstream file3;
  file3.open(path +filename[2]);
  std::ifstream file4;
  file4.open(path + filename[3]);
  std::ifstream file5;
  file5.open(path +filename[4]);

  std::ifstream* file;
  file = &file1;
  // std::getline(*file, str);
  // std::cout << str << std::endl;
  // file = &file2;
  // std::getline(*file, str);
  // std::cout << str << std::endl;
  // file = &file1;
  // std::getline(*file, str);
  // std::cout << str << std::endl;

  // std::getline(file1, str);
  // std::cout << str << std::endl;
  // std::getline(file2, str);
  // std::cout << "'" << str << "'" << std::endl;  

  std::cout << std::endl;

  while (ros::ok())
  {

    if(file->good())
    {

      // std::cout << "OPENING FILE " << file_id << std::endl;

      // if(file_id > 0)
      // {
      //   while(true)
      //   {

      //   }
      // }

      std::getline(*file, str);

      if(!str.empty())
      {

        std::stringstream part(str);

        cnt = 0;

        while(part.good())
        {
          cnt++;

          getline(part, substr, ',');

          if(!substr.empty())
          {

            // std::cout << "substr = '" << substr << "' " << substr.empty() << std::endl;

            if(cnt == 1)
            {
              point.x = stod(substr);
            }
            else if(cnt == 2)
            {
              point.y = stod(substr);
            }
            else if(cnt == 3)
            {
              point.mid = stoi(substr);
            }
            else if(cnt == 4)
            {
              point.sid = stoi(substr);
              // std::cout << "sid = " << point.sid << std::endl;
            }
          }
        }

        packet.pt.push_back(point);
        // std::cout << "\tx = " << point.x << "\ty = " << point.y << "\tmid = " << point.mid << "\tsid = " << point.sid << std::endl;

        // std::cout << "Next file is " << file_id << " " << files[file_id]->good() << std::endl;

        count++;

        if(count == count_max)
        {

          // std::cout << "count " << count << std::endl;

          count = 0;

          package.pkt.push_back(packet);
          packet.pt.clear();

          if(file_id == 0)
          {
            file = &file2;
            file_id = 1;
          }
          else if(file_id == 1)
          {
            file = &file3;
            file_id = 2;
          }
          else if(file_id == 2)
          {
            file = &file4;
            file_id = 3;
          }
          else if(file_id == 3)
          {
            file = &file5;
            file_id = 4;
          }
          else if(file_id == 4)
          {
            file = &file1;
            file_id = 0;

            // std::cout << "Published Message " << super_count << std::endl;
            chatter_pub.publish(package);
            ros::spinOnce();
            loop_rate.sleep();

            printf("\033[A\33[2KT\r");
            std::cout << "Published Message " << super_count << std::endl;

            package.pkt.clear();
            super_count++;
          }

          // if(file_id < num_files-1)
          // {
          //   file_id++;
          //   // std::cout << "Next file is " << file_id << " " << files[file_id]->good() << std::endl;
          // }
          // else
          // {
          //   // std::cout << "Published!" << std::endl;
          //   chatter_pub.publish(package);
          //   ros::spinOnce();
          //   loop_rate.sleep();

          //   package.pkt.clear();
          //   file_id = 0;
          //   next_file = true;

          //   // std::cout << "Next file is " << file_id << " " << files[file_id]->good() << std::endl;
          // }
        }
      }

    }

  }

  // while (ros::ok())
  // {

  //   int file_id = 0;

  //   // std::cout << "FILE " << files[file_id]->good() << std::endl;

  //   sleep(0.25);

  //   if(files[file_id]->good())
  //   {
  //     std::string str;
  //     std::string substr;
  //     int count = 0;
  //     int cnt;
  //     river_ros::data_pt point;
  //     river_ros::data_pkt packet;
  //     river_ros::data_pkg package;

  //     while(std::getline(*files[file_id], str))
  //     {

  //       std::cout << str << std::endl;

  //       std::stringstream part(str);

  //       cnt = 0;

  //       while(part.good())
  //       {
  //         cnt++;

  //         getline(part, substr, ',');

  //         if(cnt == 1)
  //         {
  //           point.x = std::stod(substr);
  //         }
  //         else if(cnt == 2)
  //         {
  //           point.y = std::stod(substr);
  //         }
  //         else if(cnt == 3)
  //         {
  //           point.mid = std::stoi(substr);
  //         }
  //         else if(cnt == 4)
  //         {
  //           point.sid = std::stoi(substr);
  //         }
  //       }

  //       packet.pt.push_back(point);

  //       if(count == 3)
  //       {
  //         package.pkt.push_back(packet);

  //         if(file_id < num_files-1)
  //         {
  //           file_id++;
  //           std::cout << "fild_id = " << file_id << std::endl;
  //           std::cout << "FILE file_id " << files[file_id]->good() << std::endl;
  //         }
  //         else
  //         {
  //           chatter_pub.publish(package);
  //           ros::spinOnce();
  //           loop_rate.sleep();
  //           file_id = 0;
  //         }
  //         // chatter_pub.publish(package);

  //         // ros::spinOnce();

  //         // loop_rate.sleep();

  //         count = -1;
  //         packet.pt.clear();
  //       }

  //       count++;

  //     }
  //   }
  // }



    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// // %Tag(PUBLISH)%
//     chatter_pub.publish(msg);
// // %EndTag(PUBLISH)%

// // %Tag(SPINONCE)%
//     ros::spinOnce();
// // %EndTag(SPINONCE)%

// // %Tag(RATE_SLEEP)%
//     loop_rate.sleep();
// // %EndTag(RATE_SLEEP)%
//     ++count;
  // }


  return 0;
}
// %EndTag(FULLTEXT)%