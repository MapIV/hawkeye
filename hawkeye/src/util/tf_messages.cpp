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

#include "util/tf_messages.hpp"

namespace hawkeye
{
bool TfTrajPublisher::addNewPublisher(const std::string& frame_target, const std::string& topic_name,
                                      const std::string& frame_base)
{
  if (markers_.find(frame_target) != markers_.end())
  {
    return false;
  }
  auto [iter, b] =
      markers_.insert(std::make_pair(frame_target, traj_t{ node_.advertise<nav_msgs::Path>(topic_name, 1, true), {} }));
  auto& path = iter->second.second;
  path.header.frame_id = frame_base;
  std::cout << std::quoted(frame_target) << "( from " << std::quoted(frame_base) << " ) has been added." << std::endl;
  return true;
}

bool TfTrajPublisher::remove(const std::string& frame_target)
{
  auto ret = markers_.erase(frame_target);
  return ret != 0;
}

bool TfTrajPublisher::reset(const std::string& frame_target)
{
  auto iter = markers_.find(frame_target);
  if (iter == markers_.end())
  {
    return false;
  }
  iter->second.second.poses.clear();
  return true;
}

void TfTrajPublisher::broadcastStatic(const std::string& frame_target, const tf2::Transform& tf_tf2,
                                      const ros::Time& time, const std::string& frame_base)
{
  geometry_msgs::TransformStamped tf_gm;
  tf2::convert(tf_tf2, tf_gm.transform);
  tf_gm.header.stamp = time;
  tf_gm.header.seq = 0;
  tf_gm.header.frame_id = frame_base;
  tf_gm.child_frame_id = frame_target;
  static_broadcaster_.sendTransform(tf_gm);
  std::cout << "static " << std::quoted(frame_target) << "( from " << std::quoted(frame_base) << ')'
            << " has been broadcasted." << std::endl;
}

bool TfTrajPublisher::broadcast(const std::string& frame_target, const tf2::Transform& tf_tf2, const ros::Time& time)
{
  auto iter = markers_.find(frame_target);
  if (iter == markers_.end())
  {
    std::cout << std::quoted(frame_target) << " cannot be broadcasted." << std::endl;
    return false;
  }
  geometry_msgs::TransformStamped tf_gm;
  tf2::convert(tf_tf2, tf_gm.transform);
  tf_gm.header.stamp = time;
  tf_gm.header.seq = 0;
  tf_gm.header.frame_id = iter->second.second.header.frame_id;
  tf_gm.child_frame_id = frame_target;
  dynamic_broadcaster_.sendTransform(tf_gm);

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = tf_gm.transform.translation.x;
  pose.pose.position.y = tf_gm.transform.translation.y;
  pose.pose.position.z = tf_gm.transform.translation.z;
  pose.pose.orientation = tf_gm.transform.rotation;
  pose.header = tf_gm.header;
  auto& path = iter->second.second;
  path.poses.push_back(pose);
  path.header = tf_gm.header;
  iter->second.first.publish(path);
  std::cout << std::quoted(frame_target) << " has been broadcasted." << std::endl;
  return true;
}

}  // namespace hawkeye