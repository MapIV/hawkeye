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

#include <string>

#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>

#include "hawkeye_define.hpp"

namespace hawkeye
{
inline int divFloor(int dividend, size_t divisor)
{
  if (dividend < 0)
  {
    return ~(~dividend / divisor);
  }
  return dividend / divisor;
};

class OrthoMap
{
public:
  OrthoMap(const std::string& orthomap_filename);

  inline tf2::Transform center() const
  {
    tf2::Transform center_;
    center_.setIdentity();
    center_.setOrigin(tf2::Vector3(center_x_, center_y_, 0));
    return center_;
  }

  void exportImage(const std::string& filename);

  inline double getScale() const
  {
    return scale_;
  }
  inline bool getScaled() const
  {
    return scaling_.scaled_;
  }
  inline std::pair<double, double> clampRange() const
  {
    return { scaling_.threshold_min_, scaling_.threshold_max_ };
  };
  inline bool getClassified() const
  {
    return classify_emptiness_;
  };

  const cv::Mat getImage(const tf2::Vector3& pos, int shift_x = 0, int shfit_y = 0) const;

  cv::Mat getImageRange(const tf2::Vector3& center, size_t range_x, size_t range_y) const;

  bool ckeckGridLatest(const tf2::Vector3& pos, size_t range) const;

  const nav_msgs::OccupancyGrid& getGridAround(const std_msgs::Header& header, const tf2::Vector3& pos,
                                               size_t range = 1);

private:
  inline uchar at(tf2::Vector3 pos) const
  {
    auto [w, h] = getPosOfWholeGrid(pos);
    if (!checkValid(w, h))
    {
      return 0;
    }
    auto [map_id, w2, h2] = convertPosToImage(w, h);
    if (maps_[map_id].empty())
    {
      return 0;
    }
    return maps_[map_id].at<uchar>(cv::Point(w2, h2));
  }

  inline std::pair<int, int> getPosOfWholeGrid(const tf2::Vector3& pos) const
  {
    int w = std::floor(pos.x() / scale_ + map_width_ * 0.5);
    int h = map_height_ - 1 - std::floor(pos.y() / scale_ + map_height_ * 0.5);
    return { w, h };
  }

  inline std::pair<int, int> getPosOfImageGrid(const tf2::Vector3& pos) const
  {
    auto [w, h] = getPosOfWholeGrid(pos);
    return { divFloor(w, image_width_), divFloor(h, image_height_) };
  }

  inline std::pair<int, int> getPosOfImageGrid(int w, int h) const
  {
    return { divFloor(w, image_width_), divFloor(h, image_height_) };
  }

  inline bool checkValid(int w, int h) const
  {
    return (w >= 0 && w < map_width_ && h >= 0 && h < map_height_);
  }

  inline std::tuple<size_t, size_t, size_t> convertPosToImage(int w, int h) const
  {
    size_t map_id = (h / image_height_) * map_div_x_ + (w / image_width_);
    size_t w2 = w % image_width_;
    size_t h2 = h % image_height_;
    return { map_id, w2, h2 };
  }

private:
  std::string orthomap_filename_;
  std::vector<cv::Mat> maps_;
  size_t image_width_, image_height_;
  size_t map_div_x_, map_div_y_;
  size_t map_height_, map_width_;
  double scale_;
  double center_x_;
  double center_y_;
  struct
  {
    bool scaled_;
    double threshold_min_;
    double threshold_max_;
  } scaling_;
  bool classify_emptiness_;
  struct
  {
    nav_msgs::OccupancyGrid data_;
    std::optional<std::tuple<int, int, size_t>> last_info_;
  } og_map_;
};

}  // namespace hawkeye