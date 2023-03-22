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

#include <visualization_msgs/MarkerArray.h>

#include "hawkeye_define.hpp"
#include "hawkeye_base/histogram.hpp"
#include "map/orthomap.hpp"

namespace hawkeye::hawkeye_base
{
struct HawkeyeConfig
{
  cv::Size template_size_;
  cv::Size map_size_;
  double error_rate_;
  double coeff_diminish_;
  double coeff_weight_;
  double coeff_negative_;
  double coeff_gain_;
  size_t lidar_accumulate_max_count_;
  double lidar_accumulate_max_length_;
  tf2::Transform lidar_pose_;
  double center_shift_threshold_;
  double histogram_weight_;  // for Hawkeye_WeightedShiftMode
  bool edge_copy_shift_;     // for Hawkeye_CopyShiftMode
  double intensity_accumulate_threshold_min_;
  double intensity_accumulate_threshold_max_;
  double stop_threshold_;

  static HawkeyeConfig defaultConfig();
  template <typename HawkeyeMode>
  void readYamlConfig(const std::string& yaml_filename);
};

template <typename HawkeyeMode>
class Hawkeye
{
public:
  using Config = HawkeyeConfig;

  static Config readYamlConfig(const std::string& yaml_filename);

  Hawkeye(double scale, const Config& config = Config::defaultConfig());

  tf2::Transform shift(const tf2::Transform& pose) const;

  const tf2::Transform& getLidarTf() const;

  tf2::Transform update(const tf2::Transform& pose, const PC_t::Ptr pc_ptr, OrthoMap& map, bool print_info = true);
  double getDuration() const
  {
    return essential_time_;
  };

  tf2::Transform getHistogramPeak() const;
  tf2::Transform getShftedCenter() const;
  tf2::Transform getMatchPeak() const;

  cv::Mat getTemplateImage() const;
  cv::Mat getMatchImage() const;
  cv::Mat getSubmapImage() const;
  cv::Mat getWeightImage() const;

  const visualization_msgs::MarkerArray& getHistogramMarkers(const std_msgs::Header& header, bool as_array = false);

private:
  void adjustPCs();

private:
  histogram_t histogram_;
  histogram_t shifted_histogram_;
  template_image_t template_image_;
  weighted_histogram_t weighted_histogram_;
  match_result_t match_result_;
  cv::Mat submap_;

  std::list<pose_PC_t> pc_list_;

  cv::Moments moments_;
  tf2::Vector3 offset_;
  tf2::Vector3 center_shift_;
  tf2::Vector3 average_;
  tf2::Transform odometry_;
  tf2::Transform histogram_center_;

  double essential_time_;
  size_t counter_;

  visualization_msgs::MarkerArray marker_array_;

  const Config config_;
  const double histogram_weight_;
};

extern template class Hawkeye<Hawkeye_CopyShiftMode>;
extern template class Hawkeye<Hawkeye_WeightedShiftMode>;

}  // namespace hawkeye::hawkeye_base