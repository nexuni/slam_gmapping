/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *
 */

/* Author: Brian Gerkey */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "slam_gmapping.h"

uint8_t restart_flag_;
bool isSlamOk_;

boost::mutex mutex_;

void sysCmdCallback(const std_msgs::String& sys_cmd)
  {
    if (sys_cmd.data == "reset")
    {
      boost::mutex::scoped_lock lock(mutex_);
      restart_flag_ = 1;
      isSlamOk_ = false;
    }
  }


int main(int argc, char** argv)
{
  isSlamOk_ = false;
  restart_flag_ = 1;
  ros::init(argc, argv, "slam_gmapping");
  ros::NodeHandle n;
  SlamGMapping gn;
  gn.startLiveSlam();

  ros::Subscriber sub = n.subscribe("/map_reset", 10, sysCmdCallback);

  ros::Rate rate(20);
  while (ros::ok())
  {
    if(restart_flag_ != 0)
    {
      ROS_INFO("Resetting map...");
      restart_flag_ = 0;
      gn.restart();
      ros::Duration(0.3).sleep();
      isSlamOk_ = true;
    }
    ros::MultiThreadedSpinner s(3);
    ros::spinOnce();
    rate.sleep();
  }

  return(0);
}