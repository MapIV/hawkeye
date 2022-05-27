// Copyright (c) 2022, Map IV, Inc.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <optional>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

namespace hawkeye
{
template <typename Output_>
void tf2_print(Output_& out, const std::string& name, const tf2::Transform& tf)
{
  out << name << " :" << std::endl;
  out << tf.getOrigin().x() << ' ';
  out << tf.getOrigin().y() << ' ';
  out << tf.getOrigin().z() << std::endl;
  out << tf.getRotation().x() << ' ';
  out << tf.getRotation().y() << ' ';
  out << tf.getRotation().z() << ' ';
  out << tf.getRotation().w() << std::endl;
}

class TfTrajPublisher
{
public:
  TfTrajPublisher(ros::NodeHandle& node) : node_{ node }
  {
  }

  bool remove(const std::string& frame_target);
  bool reset(const std::string& frame_target);

  bool addNewPublisher(const std::string& frame_target, const std::string& topic_name, const std::string& frame_base);

  void broadcastStatic(const std::string& frame_target, const tf2::Transform& tf_tf2, const ros::Time& time,
                       const std::string& frame_base);
  bool broadcast(const std::string& frame_target, const tf2::Transform& tf_tf2, const ros::Time& time);

private:
  ros::NodeHandle& node_;

  using traj_t = std::pair<ros::Publisher, nav_msgs::Path>;
  tf2_ros::TransformBroadcaster dynamic_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  std::map<std::string, traj_t> markers_;
};
}  // namespace hawkeye