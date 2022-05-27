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
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include "hawkeye_define.hpp"
#include "hawkeye_base/node.hpp"
#include "map/orthomap.hpp"
#include "util/arg.hpp"
#include "util/timer.hpp"
#include "util/error.hpp"
#include "util/tf_messages.hpp"
#include "util/pose_synchronize.hpp"

using namespace hawkeye;

double getYaw(const tf2::Transform& tf)
{
  double roll, pitch, yaw;
  tf.getBasis().getRPY(roll, pitch, yaw);
  return yaw;
}

class HawkeyeWSM4Node
{
public:
  HawkeyeWSM4Node(const std::string& orthomap_filename, const std::string& config_filename,
                  const std::string& lidar_topic_name,
                  const hawkeye_base::HawkeyeConfig& config = hawkeye_base::HawkeyeConfig::defaultConfig())
    : node_handle_{ "" }, counter_{ 0 }, pose_synchronizer_{ false }, times_{ 0 }, essential_times_{ 0 }
  {
    Stopwatch sw;

    std::cout << "initialization ... " << std::endl;
    sw.reset();

    // making map --------------------------------------------------
    map_.emplace(orthomap_filename);

    // setting ros publishers --------------------------------------------------
    pose_synchronizer_.addPoseSubscriber(node_handle_, "/eagleye/pose", 100);
    pose_synchronizer_.addEagleyeCheckSubscriber(node_handle_, "/eagleye/fix", 100);
    pose_synchronizer_.addTopicSubscriber(node_handle_, lidar_topic_name, 4);

    broadcaster_.emplace(node_handle_);
    broadcaster_->addNewPublisher("raw_tf", "raw_path", "local_map");
    broadcaster_->addNewPublisher("histogram_center", "histogram_center_path", "local_map");
    broadcaster_->addNewPublisher("estimated", "estimated_path", "local_map");
    broadcaster_->addNewPublisher("histogram_peak", "histogram_peak_path", "local_map");
    broadcaster_->addNewPublisher("match_peak", "match_peak_path", "local_map");

    pc_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("point_cloud", 5, false);
    map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("ortho_map", 1, false);
    histogram_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("histogram", 5, false);
    template_image_publisher_ = node_handle_.advertise<sensor_msgs::Image>("template_image", 5);
    match_image_publisher_ = node_handle_.advertise<sensor_msgs::Image>("match_image", 5);
    submap_image_publisher_ = node_handle_.advertise<sensor_msgs::Image>("submap_image", 5);
    weight_image_publisher_ = node_handle_.advertise<sensor_msgs::Image>("weight_image", 5);

    broadcaster_->broadcastStatic("local_map", map_->center(), ros::Time::now(), "map");

    // setting filter --------------------------------------------------
    hawkeye_.emplace(map_->getScale(), config);

    // other variables --------------------------------------------------
    last_updated_ = false;

    sw.count();
    std::cout << "initialization done in " << sw.get() << " ms" << std::endl;
  }

  bool runOnce(bool small_overhead = false)
  {
    auto&& synced = pose_synchronizer_.getLatest();
    if (synced)
    {
      auto& [odometry, ptr, stamp] = *synced;
      update(ptr, odometry, stamp, small_overhead);
      std::cout << "average time : " << getAverageTime() << " ms (essential : " << getAverageEssential() << " ms)"
                << std::endl;
      pose_synchronizer_.dropToLatest();
      last_updated_ = true;
    }
    else
    {
      pose_synchronizer_.dropInvalidTopics();
      last_updated_ = false;
    }
    return true;
  }

  void run(double rate = 10, bool small_overhead = false)
  {
    std::cout << "start" << std::endl;
    ros::Rate r(rate);
    while (ros::ok())
    {
      ros::spinOnce();
      runOnce(small_overhead);
      r.sleep();
    }
  }

  double getAverageTime() const
  {
    return counter_ == 0 ? -1 : (times_ / counter_);
  }
  double getAverageEssential() const
  {
    return counter_ == 0 ? -1 : (essential_times_ / counter_);
  }

private:
  void update(const sensor_msgs::PointCloud2::Ptr& ptr, const tf2::Transform& odometry, const ros::Time stamp,
              bool small_overhead = false)
  {
    Stopwatch sw, loop_sw;
    double essential_time = 0;
    std::cout << "******** " << counter_ << "th loop ********" << std::endl;
    loop_sw.reset();

    // ground segmentation --------------------------------------------------
    PC_t::Ptr ground(new PC_t);
    pcl::fromROSMsg(*ptr, *ground);

    tf2::Transform pose = map_->center().inverseTimes(odometry);
    tf2::Transform ans;

    // kanaszawa --------------------------------------------------
    ans = hawkeye_->update(pose, ground, *map_);
    essential_time += hawkeye_->getDuration();
    std::cout << "estimated pose : " << ans.getOrigin().x() << " , " << ans.getOrigin().y()
              << " ( dir : " << getYaw(ans) << " )" << std::endl;

    {
      // publish tf and traj --------------------------------------------------
      std::cout << "publish tf and traj ... " << std::endl;
      sw.reset();

      pose.getOrigin().setZ(0);
      ans.getOrigin().setZ(0);
      broadcaster_->broadcast("raw_tf", pose, stamp);
      broadcaster_->broadcast("estimated", ans, stamp);

      if (!small_overhead)
      {
        auto hcenter = hawkeye_->getShftedCenter();
        auto hpeak = hawkeye_->getHistogramPeak();
        auto mpeak = hawkeye_->getMatchPeak();
        hcenter.getOrigin().setZ(0);
        hpeak.getOrigin().setZ(0);
        mpeak.getOrigin().setZ(0);
        broadcaster_->broadcast("histogram_center", hcenter, stamp);
        broadcaster_->broadcast("histogram_peak", hpeak, stamp);
        broadcaster_->broadcast("match_peak", mpeak, stamp);
      }
      sw.count();
      std::cout << "publish tf and traj done in " << sw.get() << " ms" << std::endl;
    }
    if (!small_overhead)
    {
      {
        // publish ground point cloud --------------------------------------------------
        std::cout << "publish point cloud ... " << std::flush;
        sw.reset();
        geometry_msgs::TransformStamped tf_gm;
        PC_t shift_pc;
        sensor_msgs::PointCloud2 pcl_pc;
        tf2::convert(ans * hawkeye_->getLidarTf(), tf_gm.transform);
        // tf_gm.transform.translation.z = 0;
        pcl::transformPointCloud(*ground, shift_pc, tf2::transformToEigen(tf_gm).matrix());
        pcl::toROSMsg(shift_pc, pcl_pc);
        pcl_pc.header = ptr->header;
        pcl_pc.header.frame_id = "local_map";
        pc_publisher_.publish(pcl_pc);
        sw.count();
        std::cout << "done in " << sw.get() << " ms" << std::endl;
      }
      {
        // publish images --------------------------------------------------
        std::cout << "publish images ... " << std::flush;
        sw.reset();
        std_msgs::Header h;
        h.frame_id = "raw_tf";
        h.seq = 0;
        h.stamp = stamp;
        template_image_publisher_.publish(
            cv_bridge::CvImage(h, sensor_msgs::image_encodings::TYPE_8UC1, hawkeye_->getTemplateImage()).toImageMsg());
        match_image_publisher_.publish(
            cv_bridge::CvImage(h, sensor_msgs::image_encodings::TYPE_8UC1, hawkeye_->getMatchImage()).toImageMsg());
        submap_image_publisher_.publish(
            cv_bridge::CvImage(h, sensor_msgs::image_encodings::TYPE_8UC1, hawkeye_->getSubmapImage()).toImageMsg());
        weight_image_publisher_.publish(
            cv_bridge::CvImage(h, sensor_msgs::image_encodings::TYPE_8UC1, hawkeye_->getWeightImage()).toImageMsg());
        sw.count();
        std::cout << "done in " << sw.get() << " ms" << std::endl;
      }
    }
    {
      std_msgs::Header h;
      h.frame_id = "local_map";
      h.seq = 0;
      h.stamp = stamp;
      {
        // publish histogram --------------------------------------------------
        std::cout << "publish histogram ... " << std::flush;
        sw.reset();

        histogram_publisher_.publish(hawkeye_->getHistogramMarkers(h));

        sw.count();
        std::cout << "done in " << sw.get() << " ms" << std::endl;
      }
      {
        // publish map --------------------------------------------------
        std::cout << "publish map ... " << std::flush;
        sw.reset();

        bool update_map = !map_->ckeckGridLatest(ans.getOrigin(), 1);
        if (update_map)
        {
          map_publisher_.publish(map_->getGridAround(h, ans.getOrigin(), 1));
        }

        sw.count();
        std::cout << "done in " << sw.get() << " ms";
        if (!update_map)
        {
          std::cout << " ( not updated )";
        }
        std::cout << std::endl;
      }
    }
    loop_sw.count();
    std::cout << counter_++ << "th loop : totally " << loop_sw.get() << " ms "
              << "(essential : " << essential_time << " ms)" << std::endl;
    times_ += loop_sw.get();
    essential_times_ += essential_time;
  }

private:
  ros::NodeHandle node_handle_;
  std::optional<OrthoMap> map_;
  std::optional<hawkeye_base::Hawkeye<hawkeye_base::Hawkeye_WeightedShiftMode>> hawkeye_;

  size_t counter_;

  bool last_updated_;

  PoseSychronizer<sensor_msgs::PointCloud2::Ptr> pose_synchronizer_;

  std::optional<TfTrajPublisher> broadcaster_;
  ros::Publisher pc_publisher_;
  ros::Publisher map_publisher_;
  ros::Publisher histogram_publisher_;
  ros::Publisher template_image_publisher_;
  ros::Publisher match_image_publisher_;
  ros::Publisher submap_image_publisher_;
  ros::Publisher weight_image_publisher_;

  double times_;
  double essential_times_;

  size_t shift_count;
};

void exitArg(const std::string& message = "")
{
  std::cerr << ERROR_PREFIX << "Invalid arguments. " << message << ERROR_SUFFIX << std::endl;
  std::cerr << "./hawkeye_rt_ws"
            << " <ORTHOMAP_INFO>"
            << " <YAML_FILE>"
            << " <LIDAR_TOPIC_NAME>"
            << " | -e <ERROR_RATE>"
            << " | -d <COEFF_DIMIMISH>"
            << " | -w <COEFF_WEIGHT>"
            << " | -n <COEFF_NEGATIVE>"
            << " | -r <COEFF_GAIN>"
            << " | -a <ACCUMULATION_COUNT>"
            << " | -A <ACCUMULATION_LENGTH>"
            << " | -s <CENTER_SHIFT_THRESHOLD>"
            << " | -H <Histogram_WEIGHT>"
            << " | -o"
            << " ]*" << std::endl;
  exit(1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hawkeye_rt_ws");
  char* orthomap_filename;
  char* config_filename;
  char* lidar_topic_name;
  hawkeye_base::HawkeyeConfig config = hawkeye_base::HawkeyeConfig::defaultConfig();
  bool small_overhead = false;
  {
    argLoader param_loader(argc, argv, exitArg);
    param_loader.drop();
    orthomap_filename = param_loader.pop();
    config_filename = param_loader.pop();
    lidar_topic_name = param_loader.pop();

    std::cout << "     orthomap_filename   : " << std::quoted(orthomap_filename) << std::endl;
    std::cout << "     config_filename     : " << std::quoted(config_filename) << std::endl;
    std::cout << "     lidar_topic_name    : " << std::quoted(lidar_topic_name) << std::endl;

    config.readYamlConfig<hawkeye_base::Hawkeye_WeightedShiftMode>(config_filename);
    while (param_loader.left() > 0)
    {
      char* name = param_loader.pop();
      if (name[0] != '-')
      {
        param_loader.err();
      }
      switch (name[1])
      {
        case 'e':
        {
          config.error_rate_ = std::atof(param_loader.pop());
          break;
        }
        case 'd':
        {
          config.coeff_diminish_ = std::atof(param_loader.pop());
          break;
        }
        case 'w':
        {
          config.coeff_weight_ = std::atof(param_loader.pop());
          break;
        }
        case 'n':
        {
          config.coeff_negative_ = std::atof(param_loader.pop());
          break;
        }
        case 'r':
        {
          config.coeff_gain_ = std::atof(param_loader.pop());
          break;
        }
        case 'a':
        {
          config.lidar_accumulate_max_count_ = std::max(0l, std::atol(param_loader.pop()));
          break;
        }
        case 'A':
        {
          config.lidar_accumulate_max_length_ = std::max(0., std::atof(param_loader.pop()));
          break;
        }
        case 's':
        {
          config.center_shift_threshold_ = std::clamp(std::atof(param_loader.pop()), 0., 1.);
          break;
        }
        case 'H':
        {
          config.histogram_weight_ = std::clamp(std::atof(param_loader.pop()), 0., 1.);
          break;
        }
        case 'o':
        {
          small_overhead = true;
          break;
        }
        default:
        {
          param_loader.err();
        }
      }
    }
  }

  std::cout << "(-e) error_rate          : " << config.error_rate_ << std::endl;
  std::cout << "(-d) coeff_diminish      : " << config.coeff_diminish_ << std::endl;
  std::cout << "(-w) coeff_weight        : " << config.coeff_weight_ << std::endl;
  std::cout << "(-n) coeff_negative      : " << config.coeff_negative_ << std::endl;
  std::cout << "(-g) coeff_gain          : " << config.coeff_gain_ << std::endl;
  std::cout << "(-a) accumulation_count  : " << config.lidar_accumulate_max_count_ << std::endl;
  std::cout << "(-A) accumulation_length : " << config.lidar_accumulate_max_length_ << std::endl;
  std::cout << "(-s) center_shift_th     : " << config.center_shift_threshold_ << std::endl;
  std::cout << "(-H) histogram_weight     : " << config.histogram_weight_ << std::endl;
  std::cout << "(-o) small_overhead      : " << std::boolalpha << small_overhead << std::noboolalpha << std::endl;

  std::cout << "##### " << hawkeye_base::tmType() << " #####" << std::endl;

  HawkeyeWSM4Node node(orthomap_filename, config_filename, lidar_topic_name, config);

  node.run(10, small_overhead);

  return 0;
}