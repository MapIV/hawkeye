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

namespace hawkeye::hawkeye_base
{
template_image_t generateTemplateImage(const std::list<pose_PC_t>& pc_list, const tf2::Transform& lidar_pose,
                                       cv::Size size, scale_t scale);
template_image_t generateTemplateImage(const std::list<pose_PC_t>& pc_list, const tf2::Transform& lidar_pose,
                                       cv::Size size, scale_t scale, const tf2::Vector3& histogram_center);

template <typename HawkeyeMode>
histogram_t generateInitialDistribution(cv::Size template_size, cv::Size map_size, scale_t scale);

template <typename HawkeyeMode>
histogram_t generatePrior(const histogram_t before, const tf2::Transform& shift, size_t kernel_size,
                          double error_coeff);
template <typename HawkeyeMode>
histogram_t generatePrior(const histogram_t before, const tf2::Transform& shift, double error_coeff);

match_result_t matchTemplate(const cv::Mat map, scale_t map_scale, const template_image_t template_image,
                             bool use_mask = false);

template <typename HawkeyeMode>
histogram_t updateDistribution(const histogram_t prior, const match_result_t match_result, double diminish,
                               double weight, double negative);

//! @param weight_coeff histogram is divided by (1 - weight_coeff) + weight_coeff * weight_matrix if HawkeyeMode is
//! Hawkeye_WeightedShiftMode
template <typename HawkeyeMode>
weighted_histogram_t getWeightHistogram(const histogram_t histogram, double weight_coeff = 0);

cv::Moments getMoments(const weighted_histogram_t histogram);
cv::Moments getMoments(const match_result_t histogram);

template <typename HawkeyeMode>
inline cv::Moments getMoments(const histogram_t histogram, double weight_coeff = 0)
{
  return getMoments(getWeightHistogram<HawkeyeMode>(histogram, weight_coeff));
}

tf2::Vector3 average(const weighted_histogram_t histogram, const cv::Moments& moments);
tf2::Vector3 average(const match_result_t histogram, const cv::Moments& moments);

tf2::Vector3 updateOffset(const tf2::Vector3& prev, const tf2::Vector3& next, double coeff);

template <typename HawkeyeMode>
std::tuple<histogram_t, tf2::Vector3, tf2::Vector3> shiftCenter(const histogram_t histogram, const tf2::Vector3& center,
                                                                const tf2::Vector3& offset, double threshold_rate,
                                                                bool edge_copy_shift = true);

void setHistogramMarkers(visualization_msgs::MarkerArray& ma, const weighted_histogram_t histogram,
                         const tf2::Transform& odometry, const std_msgs::Header& header, bool as_array = false);

cv::Mat convertHistogram2CVU8C1(const cv::Mat histogram);
cv::Mat convertHistogram2CVU8C1(const weighted_histogram_t histogram);
cv::Mat convertHistogram2CVU8C1(const match_result_t histogram);

}  // namespace hawkeye::hawkeye_base