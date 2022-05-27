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

#include <boost/filesystem.hpp>
#include <tf2_eigen/tf2_eigen.h>

template <typename T>
void getFileElements(std::ifstream& ifs, const std::string& filename, T&& ref)
{
  if (!(ifs >> ref))
  {
    hawkeye::exitBadFile(filename);
  }
}

namespace hawkeye
{
constexpr int orthomap_version = 3;
OrthoMap::OrthoMap(const std::string& orthomap_filename) : orthomap_filename_{ orthomap_filename }
{
  std::string prefix;
  std::string suffix;
  bool fill_zero;
  double scale;
  double pos_x;
  double pos_y;
  int div_x;
  int div_y;
  int width;
  int height;
  int num_images;
  bool scaling_scaled;
  double scaling_threshold_min;
  double scaling_threshold_max;
  bool classify_emptiness;
  std::vector<std::pair<int, int>> index_image;
  {
    std::ifstream ifs(orthomap_filename_);
    if (!ifs)
    {
      exitOpenError(orthomap_filename_);
    }
    {
      std::string ext_name;
      int version;
      getFileElements(ifs, orthomap_filename_, ext_name);
      getFileElements(ifs, orthomap_filename_, version);
      if (ext_name != "orthomap")
      {
        exitBadFile(orthomap_filename_, "Not an orthomap file");
      }
      if (version != orthomap_version)
      {
        exitBadFile(orthomap_filename_, "version = " + std::to_string(version));
      }
    }
    getFileElements(ifs, orthomap_filename_, std::quoted(prefix));
    getFileElements(ifs, orthomap_filename_, std::quoted(suffix));
    ifs >> std::boolalpha;
    getFileElements(ifs, orthomap_filename_, fill_zero);
    ifs >> std::noboolalpha;
    getFileElements(ifs, orthomap_filename_, scale);
    getFileElements(ifs, orthomap_filename_, pos_x);
    getFileElements(ifs, orthomap_filename_, pos_y);
    getFileElements(ifs, orthomap_filename_, div_x);
    getFileElements(ifs, orthomap_filename_, div_y);
    getFileElements(ifs, orthomap_filename_, width);
    getFileElements(ifs, orthomap_filename_, height);
    getFileElements(ifs, orthomap_filename_, num_images);
    ifs >> std::boolalpha;
    getFileElements(ifs, orthomap_filename_, scaling_scaled);
    ifs >> std::noboolalpha;
    if (scaling_scaled)
    {
      getFileElements(ifs, orthomap_filename_, scaling_threshold_min);
      getFileElements(ifs, orthomap_filename_, scaling_threshold_max);
      ifs >> std::boolalpha;
      getFileElements(ifs, orthomap_filename_, classify_emptiness);
      ifs >> std::noboolalpha;
    }
    else
    {
      classify_emptiness_ = false;
    }
    {
      std::vector<int> v;
      double buf;
      while (ifs >> buf)
      {
        v.push_back(buf);
      }
      if (v.size() != div_x * div_y)
      {
        exitBadFile(orthomap_filename_);
      }
      index_image.reserve(num_images);
      for (int row = 0; row < div_y; ++row)
      {
        for (int col = 0; col < div_x; ++col)
        {
          int id = row * div_x + col;
          if (v[id] == 1)
          {
            index_image.emplace_back(row, col);
          }
        }
      }
    }
    {
      auto filename_length = boost::filesystem::path(orthomap_filename_).filename().string().size();
      prefix = orthomap_filename_.substr(0, orthomap_filename_.size() - filename_length) + prefix;
    }
  }

  scale_ = scale;
  map_div_x_ = div_x;
  map_div_y_ = div_y;
  image_height_ = height;
  image_width_ = width;
  map_height_ = height * div_y;
  map_width_ = width * div_x;
  scaling_.scaled_ = scaling_scaled;
  scaling_.threshold_min_ = scaling_threshold_min;
  scaling_.threshold_max_ = scaling_threshold_max;
  classify_emptiness_ = classify_emptiness;
  center_x_ = pos_x + ((map_width_ - 1) * 0.5) * scale;
  center_y_ = pos_y - ((map_height_ - 1) * 0.5) * scale;

  maps_.resize(div_y * div_x);

  auto filename_length = (prefix + std::to_string(num_images - 1) + suffix).size();
  auto pos_length = std::to_string(std::max(div_x, div_y) - 1).size() + 1;
  for (int i = 0; i < num_images; ++i)
  {
    std::string image_name;
    if (fill_zero)
    {
      std::ostringstream oss;
      oss << prefix << std::setw(std::to_string(num_images - 1).size()) << std::setfill('0') << i << suffix;
      image_name = oss.str();
    }
    else
    {
      image_name = prefix + std::to_string(i) + suffix;
    }
    const auto& [row_image, col_image] = index_image[i];
    std::cout << "loading " << std::setw(filename_length) << image_name;
    std::cout << " for [" << std::setw(pos_length) << row_image << ',' << std::setw(pos_length) << col_image << ']';
    std::cout << " ..." << std::flush;
    cv::Mat image = cv::imread(image_name, 0);
    if (image.empty())
    {
      exitOpenError(image_name);
    }
    if (image.size() != cv::Size(width, height))
    {
      exitBadFile(image_name, "The image size [" + std::to_string(width) + ',' + std::to_string(height) +
                                  "] is different from the size in the orthomap file.");
    }
    image.copyTo(maps_[row_image * div_x + col_image]);
    // auto& maps_map = maps_[row_image * div_x + col_image];
    // cv::Mat image_scaled = (255 - image) * 100 / (classify_emptiness ? 254. : 255.);
    // image_scaled.convertTo(maps_map, CV_8S);
    // if (classify_emptiness)
    // {
    //   bitwise_or(maps_map, -1, maps_map, image == 0);
    // }
    std::cout << " success." << std::endl;
  }
}

void OrthoMap::exportImage(const std::string& filename)
{
  cv::Mat map = cv::Mat::zeros(cv::Size(map_width_, map_height_), CV_8U);
  for (int h = 0; h < map_div_y_; ++h)
  {
    for (int w = 0; w < map_div_x_; ++w)
    {
      if (int map_id = h * map_div_x_ + w; !maps_[map_id].empty())
      {
        cv::Rect roi(cv::Point(image_width_ * w, image_height_ * h), cv::Size(image_width_, image_height_));
        maps_[map_id].copyTo(map(roi));
      }
    }
  }
  cv::imwrite(filename, map);
}

}  // namespace hawkeye