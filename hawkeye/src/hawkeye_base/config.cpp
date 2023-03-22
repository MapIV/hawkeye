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
#include "util/error.hpp"

#include <yaml-cpp/yaml.h>

namespace hawkeye::hawkeye_base
{
HawkeyeConfig HawkeyeConfig::defaultConfig()
{
  return { { 192, 192 },
           { 256, 256 },
           1. / 100,
           0.990,
           0.07,
           0.6,
           0.3,
           30,
           20.,
           tf2::Transform::getIdentity(),
           0.5,
           1,
           true,
           0,
           1,
           0.05 };
}

template <class T>
void getIfValid(const YAML::Node& node, T& val)
{
  if (node.IsDefined())
  {
    val = node.as<T>();
  }
}
template <>
void getIfValid(const YAML::Node& node, cv::Size& val)
{
  if (node.IsDefined())
  {
    std::vector<size_t> tmp = node.as<std::vector<size_t>>();
    if (tmp.size() == 2)
    {
      val.height = tmp[0];
      val.width = tmp[1];
    }
  }
}

template <typename HawkeyeMode>
void HawkeyeConfig::readYamlConfig(const std::string& yaml_filename)
{
  try
  {
    YAML::Node whole = YAML::LoadFile(yaml_filename);
    YAML::Node hawkeye_yaml = whole["hawkeye"];
    getIfValid(hawkeye_yaml["template_size"], template_size_);
    getIfValid(hawkeye_yaml["map_size"], map_size_);
    getIfValid(hawkeye_yaml["error_rate"], error_rate_);
    getIfValid(hawkeye_yaml["coeff_diminish"], coeff_diminish_);
    getIfValid(hawkeye_yaml["coeff_weight"], coeff_weight_);
    getIfValid(hawkeye_yaml["coeff_negative"], coeff_negative_);
    getIfValid(hawkeye_yaml["coeff_gain"], coeff_gain_);
    getIfValid(hawkeye_yaml["lidar_accumulate"]["max_count"], lidar_accumulate_max_count_);
    getIfValid(hawkeye_yaml["lidar_accumulate"]["max_length"], lidar_accumulate_max_length_);
    getIfValid(hawkeye_yaml["center_shift_threshold"], center_shift_threshold_);
    if constexpr (HawkeyeMode::is_weighted_)
    {
      getIfValid(hawkeye_yaml["histogram_weight"], histogram_weight_);
    }
    else
    {
      getIfValid(hawkeye_yaml["edge_copy_shift"], edge_copy_shift_);
    }
    {
      double x = 0, y = 0, z = 0;
      double roll = 0, pitch = 0, yaw = 0;
      getIfValid(whole["gnss"]["tf_x"], x);
      getIfValid(whole["gnss"]["tf_y"], y);
      getIfValid(whole["gnss"]["tf_z"], z);
      getIfValid(whole["gnss"]["tf_roll"], roll);
      getIfValid(whole["gnss"]["tf_pitch"], pitch);
      getIfValid(whole["gnss"]["tf_yaw"], yaw);
      lidar_pose_.setOrigin({ x, y, z });
      lidar_pose_.getBasis().setRPY(roll, pitch, yaw);
    }
    getIfValid(hawkeye_yaml["intensity_accumulate_threshold_max"], intensity_accumulate_threshold_max_);
    getIfValid(hawkeye_yaml["intensity_accumulate_threshold_min"], intensity_accumulate_threshold_min_);
    getIfValid(hawkeye_yaml["stop_threshold"], stop_threshold_);
  }
  catch (YAML::Exception& e)
  {
    exitBadFile(yaml_filename, e.what());
  }
}
template void HawkeyeConfig::readYamlConfig<Hawkeye_CopyShiftMode>(const std::string& yaml_filename);
template void HawkeyeConfig::readYamlConfig<Hawkeye_WeightedShiftMode>(const std::string& yaml_filename);

}  // namespace hawkeye::hawkeye_base