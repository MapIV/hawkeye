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
#include <vector>
#include <map>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hawkeye
{
template <class _Ty>
using TimeOrder_t = std::map<ros::Time, _Ty>;

inline bool comparePS(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
  return p1.header.stamp < p2.header.stamp;
}
inline bool equalPS(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
  return p1.header.stamp == p2.header.stamp;
}

enum class find_result
{
  NONE = 0,
  FOUND_SINGLE = 1 << 0,
  FOUND_MULTIPLE = 1 << 1,
  INTERPOLATABLE = 1 << 2,
  SMALLER = 1 << 3,
  LARGER = 1 << 4,
  EMPTY = 1 << 5,

  FOUND = FOUND_SINGLE | FOUND_MULTIPLE,
  NOT_FOUND = INTERPOLATABLE | SMALLER | LARGER | EMPTY,

  IP_FOUND = FOUND_SINGLE | FOUND_MULTIPLE | INTERPOLATABLE,
  IP_NOT_FOUND = SMALLER | LARGER | EMPTY,
};

inline find_result operator&(const find_result l, const find_result r)
{
  return find_result(int(l) & int(r));
}
inline find_result operator|(const find_result l, const find_result r)
{
  return find_result(int(l) | int(r));
}

std::tuple<find_result, typename std::vector<geometry_msgs::PoseStamped>::const_iterator,
           typename std::vector<geometry_msgs::PoseStamped>::const_iterator>
findBoundWithResult(const std::vector<geometry_msgs::PoseStamped>& traj_csv, const ros::Time& stamp);
std::tuple<find_result, typename TimeOrder_t<geometry_msgs::Pose>::const_iterator,
           typename TimeOrder_t<geometry_msgs::Pose>::const_iterator>
findBoundWithResult(const TimeOrder_t<geometry_msgs::Pose>& traj_csv, const ros::Time& stamp);

inline double getInterpolateRate(const ros::Time& target, const ros::Time& before, const ros::Time& after)
{
  double passed = (target - before).toNSec();
  double range = (after - before).toNSec();
  return passed / range;
}

tf2::Transform interpolatePose(const tf2::Transform& before, const tf2::Transform& after, double t);
tf2::Transform interpolatePose(const geometry_msgs::Pose& before, const geometry_msgs::Pose& after, double t);

std::optional<tf2::Transform> findPose(const std::vector<geometry_msgs::PoseStamped>& traj_csv, size_t& prev,
                                       const ros::Time& stamp);

std::optional<tf2::Transform> findPoseInterpolated(const std::vector<geometry_msgs::PoseStamped>& traj_csv,
                                                   const ros::Time& stamp);
std::optional<tf2::Transform> findPoseInterpolated(const TimeOrder_t<geometry_msgs::PoseStamped>& traj_csv,
                                                   const ros::Time& stamp);

}  // namespace hawkeye