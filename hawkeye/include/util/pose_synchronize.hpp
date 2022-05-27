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

#include <ros/ros.h>

#include "hawkeye_define.hpp"
#include "util/pose_stamped_helper.hpp"

#include <sensor_msgs/NavSatFix.h>

namespace hawkeye
{
class PoseInterpolationSubscriber
{
public:
  using this_type = PoseInterpolationSubscriber;

  PoseInterpolationSubscriber(bool check_eagleye_error = false)
    : check_eageye_error_{ check_eagleye_error }, check_eageye_error2_{ false } {};

  bool addPose(const geometry_msgs::PoseStamped& pose_stamped)
  {
    if (check_eageye_error_ && pose_stamped.pose.position.x == 0 && pose_stamped.pose.position.y == 0 &&
        pose_stamped.pose.position.z == 0)
    {
      return false;
    }
    if (check_eageye_error2_)
    {
      if (eagleye_enabled_time_.is_zero())
      {
        return false;
      }
      if (eagleye_enabled_time_ > pose_stamped.header.stamp)
      {
        std::cout << pose_stamped.header.stamp.toNSec() << " is not valid" << std::endl;
        return false;
      }
    }
    if (!pose_frame_.empty() && pose_stamped.header.frame_id != pose_frame_)
    {
      return false;
    }
    pose_map_.insert_or_assign(pose_stamped.header.stamp, pose_stamped.pose);
    return true;
  }

  void addPoseSubscriber(ros::NodeHandle& nh, const std::string& name, uint32_t queue_size,
                         const std::string& frame_name = "")
  {
    pose_subsciber_ = nh.subscribe(name, queue_size, &this_type::callbackPose, this);
    pose_frame_ = frame_name;
  }

  void addEagleyeCheckSubscriber(ros::NodeHandle& nh, const std::string& name, uint32_t queue_size)
  {
    check_eageye_error2_ = true;
    eagleye_check_subscriber_ = nh.subscribe(name, queue_size, &this_type::callbackChecker, this);
  }

protected:
  using interpolated_pose_iter = std::pair<TimeOrder_t<geometry_msgs::Pose>::const_iterator, tf2::Transform>;

  interpolated_pose_iter findForStamp_NoCheck(const ros::Time& stamp) const
  {
    auto [result, before, after] = findBoundWithResult(pose_map_, stamp);
    tf2::Transform ret;
    if (result == find_result::INTERPOLATABLE)
    {
      before--;
      ret = interpolatePose(before->second, after->second, getInterpolateRate(stamp, before->first, after->first));
    }
    else
    {
      tf2::convert(before->second, ret);
    }
    return { before, ret };
  }

private:
  void callbackPose(const geometry_msgs::PoseStamped::Ptr ptr)
  {
    addPose(*ptr);
  }

  void callbackChecker(const sensor_msgs::NavSatFix::Ptr ptr)
  {
    if (eagleye_enabled_time_.is_zero() && check_eageye_error2_ && ptr->status.service == 0)
    {
      eagleye_enabled_time_ = ptr->header.stamp;
      std::cout << "eagleye is valid after " << eagleye_enabled_time_.toNSec() << std::endl;
    }
  }

protected:
  TimeOrder_t<geometry_msgs::Pose> pose_map_;

private:
  bool check_eageye_error_;
  bool check_eageye_error2_;
  std::string pose_frame_;
  ros::Subscriber pose_subsciber_;
  ros::Subscriber eagleye_check_subscriber_;
  ros::Time eagleye_enabled_time_;
};

// --- PoseSychronizer ---
// synchronize geometry_msgs/PoseStamped and topic with header by inpterpolating poses and return tf2/Transform
template <typename _Ty, bool return_ptr>
class PoseSychronizerSub : public PoseInterpolationSubscriber
{
public:
  using this_type = PoseSychronizerSub;
  using topic_ptr_type = typename _Ty::Ptr;
  using topic_raw_type = _Ty;
  using topic_return_type = std::conditional_t<return_ptr, topic_ptr_type, topic_raw_type>;
  using return_tuple_type = std::tuple<tf2::Transform, topic_return_type, ros::Time>;

  PoseSychronizerSub(bool check_eagleye_error = false) : PoseInterpolationSubscriber{ check_eagleye_error } {};

  bool haveSynchronized() const
  {
    return !(topic_map_.empty() || pose_map_.empty() || pose_map_.begin()->first > topic_map_.rbegin()->first ||
             pose_map_.rbegin()->first < topic_map_.begin()->first);
  }

  bool addTopic(const topic_return_type& topic)
  {
    std_msgs::Header const* const header = getTopicHeaderPtr(topic);
    if (!topic_frame_.empty() && header->frame_id != topic_frame_)
    {
      return false;
    }
    topic_map_.insert_or_assign(header->stamp, topic);
    return true;
  }
  bool addTopicPtr(const topic_ptr_type& topic_ptr)
  {
    if constexpr (return_ptr)
    {
      return addTopic(topic_ptr);
    }
    else
    {
      return addTopic(*topic_ptr);
    }
  }

  void addTopicSubscriber(ros::NodeHandle& nh, const std::string& name, uint32_t queue_size,
                          const std::string& frame_name = "")
  {
    topic_subsciber_ = nh.subscribe(name, queue_size, &this_type::callbackTopic, this);
    topic_frame_ = frame_name;
  }

  std::optional<return_tuple_type> getOldest() const
  {
    if (!haveSynchronized())
    {
      return std::nullopt;
    }
    auto [pose_it, topic_it] = getOldest_NoCheck();
    return std::optional<return_tuple_type>(std::in_place, pose_it.second, topic_it->second, topic_it->first);
  }
  std::optional<return_tuple_type> getLatest() const
  {
    if (!haveSynchronized())
    {
      return std::nullopt;
    }
    auto [pose_it, topic_it] = getLatest_NoCheck();
    return std::optional<return_tuple_type>(std::in_place, pose_it.second, topic_it->second, topic_it->first);
  }

  void dropInvalidTopics()
  {
    auto topic_it = topic_map_.upper_bound(pose_map_.begin()->first);
    topic_map_.erase(topic_map_.begin(), topic_it);
  }

  void dropToOldest()
  {
    if (!haveSynchronized())
    {
      dropInvalidTopics();
      return;
    }
    auto [pose_it, topic_it] = getOldest_NoCheck();
    topic_map_.erase(topic_map_.begin(), topic_it++);
    pose_map_.erase(pose_map_.begin(), pose_it.first);
  }
  void dropToLatest()
  {
    if (!haveSynchronized())
    {
      dropInvalidTopics();
      return;
    }
    auto [pose_it, topic_it] = getLatest_NoCheck();
    topic_map_.erase(topic_map_.begin(), topic_it++);
    pose_map_.erase(pose_map_.begin(), pose_it.first);
  }

private:
  using const_topic_iter = typename TimeOrder_t<topic_return_type>::const_iterator;
  using mid_result_t = std::pair<interpolated_pose_iter, const_topic_iter>;

  static const std_msgs::Header* const getTopicHeaderPtr(const topic_ptr_type& topic)
  {
    return &(topic->header);
  }
  static const std_msgs::Header* const getTopicHeaderPtr(const topic_raw_type& topic)
  {
    return &(topic.header);
  }

  mid_result_t getOldest_NoCheck() const
  {
    auto topic_it = topic_map_.lower_bound(pose_map_.begin()->first);
    return { findForStamp_NoCheck(topic_it->first), topic_it };
  }
  mid_result_t getLatest_NoCheck() const
  {
    auto topic_it = topic_map_.upper_bound(pose_map_.rbegin()->first);
    topic_it--;
    return { findForStamp_NoCheck(topic_it->first), topic_it };
  }

  void callbackTopic(const topic_ptr_type ptr)
  {
    addTopicPtr(ptr);
  }

private:
  TimeOrder_t<topic_return_type> topic_map_;
  std::string topic_frame_;
  ros::Subscriber topic_subsciber_;
};

template <typename _Ty>
class PoseSychronizer : public PoseSychronizerSub<_Ty, false>
{
public:
  PoseSychronizer(bool check_eagleye_error = false) : PoseSychronizerSub<_Ty, false>{ check_eagleye_error }
  {
  }
};

template <typename _Ty>
class PoseSychronizer<boost::shared_ptr<_Ty>> : public PoseSychronizerSub<_Ty, true>
{
public:
  PoseSychronizer(bool check_eagleye_error = false) : PoseSychronizerSub<_Ty, true>{ check_eagleye_error }
  {
  }
};

}  // namespace hawkeye