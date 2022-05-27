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

#include <list>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "hawkeye_define.hpp"

namespace hawkeye::hawkeye_base
{
using scale_t = double;
using weight_t = double;
using h_image_t = cv::Mat;  // for histogram : CV_64F
using w_image_t = cv::Mat;  // for weight    : CV_64F
using o_image_t = cv::Mat;  // for output    : CV_8U

using template_image_t = std::tuple<o_image_t, scale_t>;
using match_result_t = std::tuple<w_image_t, weight_t, scale_t>;
using weighted_histogram_t = std::tuple<h_image_t, scale_t>;

using pose_PC_t = std::pair<PC_t::Ptr, tf2::Transform>;

struct Hawkeye_CopyShiftMode
{
  static constexpr bool is_weighted_ = false;
};

struct Hawkeye_WeightedShiftMode
{
  static constexpr bool is_weighted_ = true;
};

using histogram_t = std::tuple<h_image_t, w_image_t, scale_t>;

}  // namespace hawkeye::hawkeye_base