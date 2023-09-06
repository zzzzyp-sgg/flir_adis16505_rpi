/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie
Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution
Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.
Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
   @file node.cpp
   @author Chad Rockey
   @date July 13, 2011
   @brief ROS node to wrap the nodelet for standalone rosrun execution

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "ros/ros.h"
#include <nodelet/loader.h>

#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spinnaker_camera_node");

  // This is code based nodelet loading, the preferred nodelet launching is done through roslaunch
  /**
   * Nodelet节点与传统的ros节点有点不一样，不一样在于，节点的源文件cpp里面并没有main函数。
   * 其特点是：是一个类(Class)的形式。
   * 所以，编译的时候，并不是编译成可执行文件，而是编译成库文件
  */
  nodelet::Loader nodelet;
  /**
   * 创建了一个 nodelet::M_string 类型的对象 remap，它将用于存储ROS节点的重新映射信息。
   * ROS节点可以通过重新映射来改变它们的话题、服务等名称，以适应不同的运行环境。
  */
  nodelet::M_string remap(ros::names::getRemappings());
  /* 存储节点的命令行参数 */
  nodelet::V_string nargv;
  /* 获取节点名字 */
  std::string nodelet_name = ros::this_node::getName();
  /* 加载节点 args: 名字 节点类型 重映射信息 命令行参数 */
  nodelet.load(nodelet_name, "spinnaker_camera_driver/SpinnakerCameraNodelet", remap, nargv);

  ros::spin();

  return 0;
}
