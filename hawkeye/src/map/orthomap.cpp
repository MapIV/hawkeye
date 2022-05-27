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

#include "map/orthomap.hpp"
#include "util/error.hpp"
#include "util/timer.hpp"

namespace hawkeye
{
const cv::Mat OrthoMap::getImage(const tf2::Vector3& pos, int shift_x, int shift_y) const
{
  auto [w, h] = getPosOfWholeGrid(pos);
  w += shift_x * int(image_width_);
  h -= shift_y * int(image_height_);
  if (!checkValid(w, h))
  {
    return cv::Mat();
  }
  return maps_[std::get<0>(convertPosToImage(w, h))];
}

cv::Mat OrthoMap::getImageRange(const tf2::Vector3& center, size_t range_x, size_t range_y) const
{
  cv::Mat ret(cv::Size(range_x, range_y), CV_8U, cv::Scalar(0));
  int w_min, h_min;
  {
    auto [w, h] = getPosOfWholeGrid(center);
    w_min = w - range_x / 2;
    h_min = h - range_y / 2;
  }
  int w_max = w_min + range_x;
  int h_max = h_min + range_y;
  for (int wi = divFloor(w_min, int(image_width_)); wi * int(image_width_) < w_max; ++wi)
  {
    if (wi < 0)
    {
      continue;
    }
    if (wi >= map_div_x_)
    {
      break;
    }
    const int rect_w_min = std::max<int>(w_min, wi * int(image_width_));
    const int rect_w_size = std::min<int>(w_max, (wi + 1) * int(image_width_)) - rect_w_min;
    for (int hi = divFloor(h_min, int(image_height_)); hi * int(image_height_) < h_max; ++hi)
    {
      if (hi < 0)
      {
        continue;
      }
      if (hi >= map_div_y_)
      {
        break;
      }
      const int rect_h_min = std::max<int>(h_min, hi * int(image_height_));
      const int rect_h_size = std::min<int>(h_max, (hi + 1) * int(image_height_)) - rect_h_min;
      const cv::Mat map = maps_[wi + hi * map_div_x_];
      if (map.empty())
      {
        continue;
      }
      const cv::Size roi_size(rect_w_size, rect_h_size);
      const cv::Rect roi_ret(cv::Point(rect_w_min - w_min, rect_h_min - h_min), roi_size);
      const cv::Rect roi_map(cv::Point(rect_w_min - wi * int(image_width_), rect_h_min - hi * int(image_height_)),
                             roi_size);
      map(roi_map).copyTo(ret(roi_ret));
    }
  }
  return ret;
}

bool OrthoMap::ckeckGridLatest(const tf2::Vector3& pos, size_t range) const
{
  if (og_map_.last_info_.has_value())
  {
    auto [center_w, center_h] = getPosOfImageGrid(pos);
    return *og_map_.last_info_ == std::make_tuple(center_w, center_h, range);
  }
  return false;
}

const nav_msgs::OccupancyGrid& OrthoMap::getGridAround(const std_msgs::Header& header, const tf2::Vector3& pos,
                                                       size_t range)
{
  og_map_.data_.header = header;
  og_map_.data_.info.origin.position.z = pos.z();
  auto [center_w, center_h] = getPosOfImageGrid(pos);
  if (og_map_.last_info_.has_value())
  {
    if (*og_map_.last_info_ == std::make_tuple(center_w, center_h, range))
    {
      return og_map_.data_;
    }
    *og_map_.last_info_ = std::make_tuple(center_w, center_h, range);
  }
  else
  {
    og_map_.last_info_.emplace(center_w, center_h, range);
  }
  size_t range2 = 2 * range + 1u;
  int rangei = int(range);
  size_t width = range2 * image_width_;
  size_t height = range2 * image_height_;
  {
    auto& info = og_map_.data_.info;
    info.width = width;
    info.height = height;
    info.resolution = scale_;
    double diff_x = center_w - rangei - map_div_x_ * 0.5;
    double diff_y = center_h + (rangei + 1) - map_div_y_ * 0.5;
    info.origin.position.x = diff_x * image_width_ * scale_ - scale_ / 2;
    info.origin.position.y = -diff_y * image_height_ * scale_ + scale_ / 2;
    info.origin.position.z = pos.z();
    info.origin.orientation.x = 0;
    info.origin.orientation.y = 0;
    info.origin.orientation.z = 0;
    info.origin.orientation.w = 1;

    og_map_.data_.data.clear();
    og_map_.data_.data.resize(width * height, -1);
  }
  {
    for (int y = -rangei; y <= rangei; ++y)
    {
      for (int x = -rangei; x <= rangei; ++x)
      {
        if (const cv::Mat m = getImage(pos, x, y); !m.empty())
        {
          int index_base = width * (image_height_ * (y + rangei + 1) - 1) + image_width_ * (x + rangei);
          if (!classify_emptiness_)
          {
            m.forEach<uchar>([this, &index_base, &width](uchar c, const int* position) {
              auto& h = position[0];
              auto& w = position[1];
              og_map_.data_.data[index_base + w - h * width] = (255 - c) * 100 / 255.;
            });
          }
          else
          {
            m.forEach<uchar>([this, &index_base, &width](uchar c, const int* position) {
              if (c != 0)
              {
                auto& h = position[0];
                auto& w = position[1];
                og_map_.data_.data[index_base + w - h * width] = (255 - c) * 100 / 254.;
              }
            });
          }
        }
      }
    }
  }
  return og_map_.data_;
}
}  // namespace hawkeye