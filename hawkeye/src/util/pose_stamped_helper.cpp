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

#include "util/pose_stamped_helper.hpp"

#include <algorithm>
#include <iterator>

namespace hawkeye
{
template <typename Iter_>
inline find_result checkResult(const Iter_& bg, const Iter_& ed, const Iter_& lb, const Iter_& ub)
{
  if (ub == std::next(lb))
  {
    return find_result::FOUND_SINGLE;
  }
  else if (ub != lb)
  {
    return find_result::FOUND_MULTIPLE;
  }
  else if (bg == ed)
  {
    return find_result::EMPTY;
  }
  else if (lb == bg)
  {
    return find_result::SMALLER;
  }
  else if (lb == ed)
  {
    return find_result::SMALLER;
  }
  else
  {
    return find_result::INTERPOLATABLE;
  }
}

std::tuple<find_result, typename std::vector<geometry_msgs::PoseStamped>::const_iterator,
           typename std::vector<geometry_msgs::PoseStamped>::const_iterator>
findBoundWithResult(const std::vector<geometry_msgs::PoseStamped>& traj_csv, const ros::Time& stamp)
{
  geometry_msgs::PoseStamped tmp;
  tmp.header.stamp = stamp;
  auto lb = std::lower_bound(traj_csv.begin(), traj_csv.end(), tmp, comparePS);
  auto ub = std::upper_bound(traj_csv.begin(), traj_csv.end(), tmp, comparePS);
  find_result result = checkResult(traj_csv.cbegin(), traj_csv.cend(), lb, ub);
  return { result, lb, ub };
}
std::tuple<find_result, typename TimeOrder_t<geometry_msgs::Pose>::const_iterator,
           typename TimeOrder_t<geometry_msgs::Pose>::const_iterator>
findBoundWithResult(const TimeOrder_t<geometry_msgs::Pose>& traj_csv, const ros::Time& stamp)
{
  auto lb = traj_csv.lower_bound(stamp);
  auto ub = traj_csv.upper_bound(stamp);
  find_result result = checkResult(traj_csv.cbegin(), traj_csv.cend(), lb, ub);
  return { result, lb, ub };
}

tf2::Transform interpolatePose(const tf2::Transform& before, const tf2::Transform& after, double t)
{
  tf2::Transform pose;
  pose.setOrigin(before.getOrigin() * (1 - t) + after.getOrigin() * t);
  pose.setRotation(tf2::slerp(before.getRotation(), after.getRotation(), t));
  return pose;
}

tf2::Transform interpolatePose(const geometry_msgs::Pose& before, const geometry_msgs::Pose& after, double t)
{
  tf2::Transform before_tf, after_tf;
  tf2::convert(before, before_tf);
  tf2::convert(after, after_tf);
  return interpolatePose(before_tf, after_tf, t);
}

std::optional<tf2::Transform> findPoseInterpolated(const std::vector<geometry_msgs::PoseStamped>& traj_csv,
                                                   const ros::Time& stamp)
{
  auto [result, lb, ub] = findBoundWithResult(traj_csv, stamp);
  if ((result & find_result::FOUND) != find_result::NONE)
  {
    tf2::Transform pose;
    tf2::convert(lb->pose, pose);
    return pose;
  }
  else if (result == find_result::INTERPOLATABLE)
  {
    lb--;
    return interpolatePose(lb->pose, ub->pose, getInterpolateRate(stamp, lb->header.stamp, ub->header.stamp));
  }
  return std::nullopt;
}
std::optional<tf2::Transform> findPoseInterpolated(const TimeOrder_t<geometry_msgs::Pose>& traj_csv,
                                                   const ros::Time& stamp)
{
  auto [result, lb, ub] = findBoundWithResult(traj_csv, stamp);
  if ((result & find_result::FOUND) != find_result::NONE)
  {
    tf2::Transform pose;
    tf2::convert(lb->second, pose);
    return pose;
  }
  else if (result == find_result::INTERPOLATABLE)
  {
    lb--;
    return interpolatePose(lb->second, ub->second, getInterpolateRate(stamp, lb->first, ub->first));
  }
  return std::nullopt;
}

std::optional<tf2::Transform> findPose(const std::vector<geometry_msgs::PoseStamped>& traj_csv, size_t& prev,
                                       const ros::Time& stamp)
{
  size_t id;
  if (traj_csv[prev + 1].header.stamp == stamp)
  {
    id = prev + 1;
    prev = id;
  }
  else
  {
    size_t l = prev, u = traj_csv.size();
    while (u - l > 1)
    {
      auto& s = traj_csv[l + (u - l - 1) / 2].header.stamp;
      if (s == stamp)
      {
        break;
      }
      else if (s < stamp)
      {
        l = l + (u - l - 1) / 2 + 1;
      }
      else
      {
        u = l + (u - l - 1) / 2;
      }
    }
    if (u == l)
    {
      u++;
    }
    id = l + (u - l - 1) / 2;
    prev = id;
    if (traj_csv[id].header.stamp != stamp)
    {
      return std::nullopt;
    }
  }
  tf2::Transform pose;
  tf2::convert(traj_csv[id].pose, pose);
  return pose;
}

}  // namespace hawkeye