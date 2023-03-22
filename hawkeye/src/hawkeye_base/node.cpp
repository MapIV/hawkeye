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

#include <iostream>

#include "hawkeye_base/node.hpp"
#include "hawkeye_base/base.hpp"
#include "util/timer.hpp"

namespace hawkeye::hawkeye_base
{
constexpr bool use_cv_tm_mask_ = true;

double getTrueHistogramWeight(double histogram_weight, double coeff_diminish, double coeff_weight,
                              double coeff_negative)
{
  if (histogram_weight == 0)
  {
    return 0;
  }
  double max_val = (coeff_weight - coeff_negative) / (1 - coeff_diminish);
  return histogram_weight / (max_val * (1 - histogram_weight) + histogram_weight);
}

template <typename HawkeyeMode>
typename Hawkeye<HawkeyeMode>::Config Hawkeye<HawkeyeMode>::readYamlConfig(const std::string& yaml_filename)
{
  Config conf = Config::defaultConfig();
  conf.readYamlConfig<HawkeyeMode>(yaml_filename);
  return conf;
}

template <typename HawkeyeMode>
Hawkeye<HawkeyeMode>::Hawkeye(double scale, const Config& config)
  : essential_time_{ 0 }
  , offset_{ 0, 0, 0 }
  , center_shift_{ 0, 0, 0 }
  , config_{ config }
  , counter_{ 0 }
  , histogram_weight_{ getTrueHistogramWeight(config.histogram_weight_, config.coeff_diminish_, config.coeff_weight_,
                                              config.coeff_negative_) }
{
  histogram_ = generateInitialDistribution<HawkeyeMode>(config_.template_size_, config_.map_size_, scale);
  if constexpr (HawkeyeMode::is_weighted_)
  {
    std::cout << "histogram weight used in calculation : " << histogram_weight_ << std::endl;
  }
}

template <typename HawkeyeMode>
tf2::Transform Hawkeye<HawkeyeMode>::shift(const tf2::Transform& pose) const
{
  if (counter_ == 0)
  {
    return pose;
  }
  tf2::Transform ret = pose;
  ret.getOrigin() += offset_ + center_shift_;
  return ret;
}

template <typename HawkeyeMode>
const tf2::Transform& Hawkeye<HawkeyeMode>::getLidarTf() const
{
  return config_.lidar_pose_;
}

template <typename HawkeyeMode>
void Hawkeye<HawkeyeMode>::adjustPCs()
{
  if (config_.lidar_accumulate_max_length_ > 0. && pc_list_.size() > 1)
  {
    auto last_pos = pc_list_.back().second.getOrigin();
    pc_list_.remove_if([&last_pos, threshold = config_.lidar_accumulate_max_length_](auto p) {
      return p.second.getOrigin().distance(last_pos) > threshold;
    });
    if (config_.lidar_accumulate_max_count_ > 0)
    {
      while (pc_list_.size() > config_.lidar_accumulate_max_count_)
      {
        double min_d = config_.lidar_accumulate_max_length_ * 2;
        auto min_it = pc_list_.begin();
        for (auto it = pc_list_.begin(), next = std::next(it); next != pc_list_.end(); ++it, ++next)
        {
          if (double d = tf2::tf2Distance(it->second.getOrigin(), next->second.getOrigin()); d < min_d)
          {
            min_d = d;
            min_it = it;
          }
        }
        pc_list_.erase(min_it);
      }
    }
  }
  else if (config_.lidar_accumulate_max_count_ > 0)
  {
    while (config_.lidar_accumulate_max_count_ < pc_list_.size())
    {
      pc_list_.pop_front();
    }
  }
}

template <typename HawkeyeMode>
tf2::Transform Hawkeye<HawkeyeMode>::update(const tf2::Transform& pose, const PC_t::Ptr pc_ptr, OrthoMap& map,
                                            bool print_info)
{
  Stopwatch sw;
  essential_time_ = 0;
  tf2::Transform ret;

  // updating point cloud list --------------------------------------------------
  if (print_info)
  {
    std::cout << "updating point cloud list ... " << std::flush;
  }
  sw.reset();
  {
    pc_list_.emplace_back(pc_ptr, pose);
    adjustPCs();
  }
  essential_time_ += sw.count();
  if (print_info)
  {
    std::cout << "done in " << sw.get() << " ms ( size : " << pc_list_.size() << " )" << std::endl;
  }

  // skip if stopping --------------------------------------------------
  if ((odometry_.getOrigin() - pose.getOrigin()).length() < config_.stop_threshold_ && counter_ != 0)
  {
    if (print_info)
    {
      std::cout << "skip histgram update while stopping" << std::endl;
    }
    ret = pose;
    ret.getOrigin() += center_shift_ + offset_;
  }
  else
  {
    if (counter_ != 0)
    {
      // generating prior --------------------------------------------------
      if (print_info)
      {
        std::cout << "generating prior distribution ... " << std::flush;
      }
      sw.reset();
      histogram_ = hawkeye_base::generatePrior<HawkeyeMode>(shifted_histogram_, odometry_.inverseTimes(pose),
                                                            config_.error_rate_);
      essential_time_ += sw.count();
      if (print_info)
      {
        std::cout << "done in " << sw.get() << " ms" << std::endl;
      }
    }

    // generating template image --------------------------------------------------
    if (print_info)
    {
      std::cout << "generating template image ... " << std::flush;
    }
    sw.reset();
    histogram_center_ = pose;
    histogram_center_.getOrigin() += center_shift_;
    template_image_ = generateTemplateImage(
        pc_list_, config_.lidar_pose_, config_.template_size_, std::get<2>(histogram_), pose.getOrigin(),
        config_.intensity_accumulate_threshold_min_, config_.intensity_accumulate_threshold_max_);
    essential_time_ += sw.count();
    if (print_info)
    {
      std::cout << "done in " << sw.get() << " ms" << std::endl;
    }

    // generating map image --------------------------------------------------
    if (print_info)
    {
      std::cout << "generating map image ... " << std::flush;
    }
    sw.reset();
    submap_ = map.getImageRange(histogram_center_.getOrigin(), config_.map_size_.width, config_.map_size_.height);
    essential_time_ += sw.count();
    if (print_info)
    {
      std::cout << "done in " << sw.get() << " ms" << std::endl;
      std::cout << "histogram center : " << histogram_center_.getOrigin().x() << ", "
                << histogram_center_.getOrigin().y();
      std::cout << " ( " << center_shift_.x() << ", " << center_shift_.y();
      std::cout << " / " << offset_.x() << ", " << offset_.y() << " )" << std::endl;
    }

    // matching --------------------------------------------------
    if (print_info)
    {
      std::cout << "matching ... " << std::flush;
    }
    sw.reset();
    match_result_ = matchTemplate(submap_, map.getScale(), template_image_, true);
    essential_time_ += sw.count();
    if (print_info)
    {
      double min, max;
      cv::minMaxIdx(std::get<0>(match_result_), &min, &max);
      std::cout << "done in " << sw.get() << " ms ... [ " << min << " , " << max << " ]" << std::endl;
    }

    // updating distribution --------------------------------------------------
    if (print_info)
    {
      std::cout << "updating distribution ... " << std::flush;
    }
    sw.reset();
    histogram_ = updateDistribution<HawkeyeMode>(histogram_, match_result_, config_.coeff_diminish_,
                                                 config_.coeff_weight_, config_.coeff_negative_);
    essential_time_ += sw.count();
    if (print_info)
    {
      std::cout << "done in " << sw.get() << " ms" << std::endl;
    }

    // updating offset --------------------------------------------------
    if (print_info)
    {
      std::cout << "updating offset ... " << std::flush;
    }
    sw.reset();
    weighted_histogram_ = getWeightHistogram<HawkeyeMode>(histogram_, config_.histogram_weight_);
    moments_ = getMoments(weighted_histogram_);
    average_ = average(weighted_histogram_, moments_);
    offset_ = updateOffset(offset_, average_, config_.coeff_gain_);
    essential_time_ += sw.count();
    if (print_info)
    {
      std::cout << "done in " << sw.get() << " ms "
                << "( " << average_.x() << ", " << average_.y() << " "
                << "/ " << offset_.x() << ", " << offset_.y() << " )" << std::endl;
    }

    // updating histogram center --------------------------------------------------
    if (print_info)
    {
      std::cout << "updating histogram center ... " << std::flush;
    }
    sw.reset();
    tf2::Vector3 new_offset;
    std::tie(shifted_histogram_, center_shift_, new_offset) = shiftCenter<HawkeyeMode>(
        histogram_, center_shift_, offset_, config_.center_shift_threshold_, config_.edge_copy_shift_);
    essential_time_ += sw.count();
    if (print_info)
    {
      std::cout << "done in " << sw.get() << " ms" << std::endl;
    }
    odometry_ = pose;
    ret = histogram_center_;
    ret.setOrigin(ret.getOrigin() + offset_);
    offset_ = new_offset;
  }

  ++counter_;
  return ret;
}

template <typename HawkeyeMode>
tf2::Transform Hawkeye<HawkeyeMode>::getHistogramPeak() const
{
  tf2::Transform ret = histogram_center_;
  ret.getOrigin() += average_;
  return ret;
}

template <typename HawkeyeMode>
tf2::Transform Hawkeye<HawkeyeMode>::getShftedCenter() const
{
  return histogram_center_;
}

template <typename HawkeyeMode>
tf2::Transform Hawkeye<HawkeyeMode>::getMatchPeak() const
{
  auto& [hist, weight, scale] = match_result_;
  cv::Mat positive(hist.size(), hist.type());
  cv::threshold(hist, positive, 0, 0, cv::THRESH_TOZERO);
  tf2::Transform ret = histogram_center_;
  match_result_t positive_result(positive, weight, scale);
  ret.getOrigin() += average(positive_result, getMoments(positive_result));
  return ret;
}

template <typename HawkeyeMode>
cv::Mat Hawkeye<HawkeyeMode>::getTemplateImage() const
{
  return std::get<0>(template_image_);
}
template <typename HawkeyeMode>
cv::Mat Hawkeye<HawkeyeMode>::getMatchImage() const
{
  return convertHistogram2CVU8C1(match_result_);
}
template <typename HawkeyeMode>
cv::Mat Hawkeye<HawkeyeMode>::getSubmapImage() const
{
  return submap_;
}
template <typename HawkeyeMode>
cv::Mat Hawkeye<HawkeyeMode>::getWeightImage() const
{
  return convertHistogram2CVU8C1(std::get<1>(histogram_));
}

template <typename HawkeyeMode>
const visualization_msgs::MarkerArray& Hawkeye<HawkeyeMode>::getHistogramMarkers(const std_msgs::Header& header,
                                                                                 bool as_array)
{
  setHistogramMarkers(marker_array_, weighted_histogram_, histogram_center_, header, as_array);
  return marker_array_;
}

template class Hawkeye<Hawkeye_CopyShiftMode>;
template class Hawkeye<Hawkeye_WeightedShiftMode>;

}  // namespace hawkeye::hawkeye_base