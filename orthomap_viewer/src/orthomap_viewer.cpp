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

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <string>
#include <vector>
#include <optional>
#include <iostream>
#include <iomanip>

#define SHOW_PARAM(val) std::cout << #val " : " << val << std::endl
#define SHOW_PARAM2(val, exp) std::cout << #val " : " << exp << std::endl

constexpr int valid_version = 3;

void exitArg(const std::string& message = "")
{
  std::cerr << "\033[1;31mError: Invalid arguments. " << message << "\033[0m" << std::endl;
  std::cerr << "./ortho_publisher <ORTHOMAP_INFO> ["
            << " -t <TOPIC_NAME> |"
            << " -p <PCD_XYZI_FILE> |"
            << " -f <FRAME_NAME> ]*" << std::endl;
  exit(1);
}

void exitBadFile(const std::string& filename, const std::string& message = "")
{
  std::cerr << "\033[1;31mError: Bad file: " << filename << ' ' << message << "\033[0m" << std::endl;
  exit(2);
}

template <typename T>
void getFileElements(std::ifstream& ifs, const std::string& filename, T&& ref)
{
  if (!(ifs >> ref))
  {
    exitBadFile(filename);
  }
}

std::optional<nav_msgs::OccupancyGrid> draw_one(const std::string& topic_name, const std::string& tif_name, int seq);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orthomap_view");
  std::string input_file;
  std::string topic_name = "ortho_map";
  bool topic_name_param;
  std::optional<std::string> pcd_name;
  std::optional<std::string> frame_name;
  if (argc < 2)
  {
    exitArg("argc: " + std::to_string(argc));
  }
  input_file = argv[1];
  for (int i = 2; i < argc;)
  {
    if (!strcmp(argv[i], "-t"))
    {
      if (topic_name_param || argc < i + 2)
      {
        exitArg();
      }
      topic_name_param = true;
      topic_name = argv[i + 1];
      i += 2;
    }
    else if (!strcmp(argv[i], "-p"))
    {
      if (pcd_name || argc < i + 2)
      {
        exitArg();
      }
      pcd_name = argv[i + 1];
      i += 2;
    }
    else if (!strcmp(argv[i], "-f"))
    {
      if (frame_name || argc < i + 2)
      {
        exitArg();
      }
      frame_name = argv[i + 1];
      i += 2;
    }
    else
    {
      exitArg();
    }
  }
  if (!frame_name)
  {
    frame_name = "map";
  }
  SHOW_PARAM(topic_name);
  SHOW_PARAM(input_file);
  if (pcd_name)
  {
    SHOW_PARAM2(pcd_name, *pcd_name);
  }
  SHOW_PARAM2(frame_name, *frame_name);

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
  bool scaled;
  double threshold_min;
  double threshold_max;
  bool classify_emptiness;
  std::vector<std::pair<int, int>> index_image;
  {
    std::ifstream ifs(input_file);
    if (!ifs)
    {
      std::cerr << "\033[1;31mError: Could not open " << input_file << "\033[0m" << std::endl;
      exit(2);
    }
    {
      std::string ext_name;
      int version;
      getFileElements(ifs, input_file, ext_name);
      getFileElements(ifs, input_file, version);
      if (ext_name != "orthomap")
      {
        exitBadFile(input_file, "ext_name = " + ext_name);
      }
      if (version != valid_version)
      {
        exitBadFile(input_file, "version = " + std::to_string(version));
      }
    }
    getFileElements(ifs, input_file, std::quoted(prefix));
    getFileElements(ifs, input_file, std::quoted(suffix));
    ifs >> std::boolalpha;
    getFileElements(ifs, input_file, fill_zero);
    ifs >> std::noboolalpha;
    getFileElements(ifs, input_file, scale);
    getFileElements(ifs, input_file, pos_x);
    getFileElements(ifs, input_file, pos_y);
    getFileElements(ifs, input_file, div_x);
    getFileElements(ifs, input_file, div_y);
    getFileElements(ifs, input_file, width);
    getFileElements(ifs, input_file, height);
    getFileElements(ifs, input_file, num_images);
    ifs >> std::boolalpha;
    getFileElements(ifs, input_file, scaled);
    ifs >> std::noboolalpha;
    if (scaled)
    {
      getFileElements(ifs, input_file, threshold_min);
      getFileElements(ifs, input_file, threshold_min);
      ifs >> std::boolalpha;
      getFileElements(ifs, input_file, classify_emptiness);
      ifs >> std::noboolalpha;
    }
    else
    {
      classify_emptiness = false;
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
        exitBadFile(input_file, " (" + std::to_string(v.size()) + ")");
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
      auto filename_length = boost::filesystem::path(input_file).filename().string().size();
      prefix = input_file.substr(0, input_file.size() - filename_length) + prefix;
    }
  }

  SHOW_PARAM2(prefix, std::quoted(prefix));
  SHOW_PARAM2(suffix, std::quoted(suffix));
  SHOW_PARAM2(fill_zero, std::boolalpha << fill_zero << std::noboolalpha);
  SHOW_PARAM(scale);
  SHOW_PARAM(pos_x);
  SHOW_PARAM(pos_y);
  SHOW_PARAM(div_x);
  SHOW_PARAM(div_y);
  SHOW_PARAM(width);
  SHOW_PARAM(height);
  SHOW_PARAM(num_images);
  SHOW_PARAM2(classify_emptiness, std::boolalpha << classify_emptiness << std::noboolalpha);

  ros::NodeHandle n("~");
  ros::Publisher lane_publisher = n.advertise<nav_msgs::OccupancyGrid>("ortho_images", 1, true);
  nav_msgs::OccupancyGrid grid;

  grid.info.width = width * div_x;
  grid.info.height = height * div_y;
  grid.info.resolution = scale;
  grid.data.resize(grid.info.width * grid.info.height, -1);
  grid.header.frame_id = *frame_name;

  grid.info.origin.position.x = pos_x - scale / 2;
  grid.info.origin.position.y = pos_y + scale / 2 - height * div_y * scale;
  grid.info.origin.position.z = 0;
  grid.info.origin.orientation.x = 0;
  grid.info.origin.orientation.y = 0;
  grid.info.origin.orientation.z = 0;
  grid.info.origin.orientation.w = 1;

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
    if (image.data == nullptr)
    {
      std::cerr << "\033[1;31mError: Could not open " << image_name << "\033[0m" << std::endl;
      exit(2);
    }
    const int row_base = row_image * height;
    const int row_max = height * div_y - 1;
    const int col_base = col_image * width;
    const int line_length = width * div_x;
    for (int row = 0; row < image.rows; ++row)
    {
      for (int col = 0; col < image.cols; ++col)
      {
        auto index = (row_max - (row + row_base)) * line_length + (col + col_base);
        if (classify_emptiness)
        {
          if (image.at<uchar>(row, col) != 0)
          {
            grid.data[index] = (255 - image.at<uchar>(row, col)) * 100 / 254.;
          }
        }
        else
        {
          grid.data[index] = (255 - image.at<uchar>(row, col)) * 100 / 255.;
        }
      }
    }
    std::cout << " success." << std::endl;
  }
  lane_publisher.publish(grid);
  pcl::PointCloud<pcl::PointXYZI> pc;
  ros::Publisher pc_pub;
  if (pcd_name)
  {
    if (pcl::io::loadPCDFile(*pcd_name, pc) < 0)
    {
      std::cerr << "\033[1;31mError: Could not open " << *pcd_name << " as pcl::PointCloud<pcl::PointXYZI> \033[0m"
                << std::endl;
      exit(2);
    }
    for (auto& p : pc)
    {
      p.z = 0.000001;
    }
    pc_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI>>("pcd_points", 1, true);
    pc.header.frame_id = *frame_name;
    pc_pub.publish(pc);
    std::cout << "publish point cloud" << std::endl;
  }
  std::cout << "finish and spin" << std::endl;
  ros::spin();
  std::cout << std::endl;
  return 0;
}