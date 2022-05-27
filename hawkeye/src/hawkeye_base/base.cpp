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

#include "hawkeye_base/base.hpp"
#include "util/error.hpp"

namespace hawkeye::hawkeye_base
{
template_image_t generateTemplateImage(const std::list<pose_PC_t>& pc_list, const tf2::Transform& lidar_pose,
                                       cv::Size size, scale_t scale)
{
  if (pc_list.empty())
  {
    std::cerr << ERROR_PREFIX << "No point cloud." << ERROR_SUFFIX << std::endl;
    exit(4);
  }
  return generateTemplateImage(pc_list, lidar_pose, size, scale, pc_list.back().second.getOrigin());
}

template_image_t generateTemplateImage(const std::list<pose_PC_t>& pc_list, const tf2::Transform& lidar_pose,
                                       cv::Size size, scale_t scale, const tf2::Vector3& histogram_center)
{
  cv::Mat ret(size, CV_8U, cv::Scalar(0));
  for (auto& [ptr, tf] : pc_list)
  {
    tf2::Transform shift_rotate = tf * lidar_pose;
    shift_rotate.getOrigin() -= histogram_center;
    for (auto& p : *ptr)
    {
      tf2::Vector3 tf = shift_rotate * tf2::Vector3(p.x, p.y, 0);
      int w = std::floor(tf.x() / scale + size.width * 0.5);
      int h = size.height - 1 - std::floor(tf.y() / scale + size.height * 0.5);
      if (w >= 0 && w < size.width && h >= 0 && h < size.height)
      {
        ret.at<uchar>(cv::Point(w, h)) = std::clamp<double>(p.intensity, 0, 255) * 254. / 255 + 1;
      }
    }
  }
  return { ret, scale };
}

template <>
histogram_t generateInitialDistribution<Hawkeye_CopyShiftMode>(cv::Size template_size, cv::Size map_size, scale_t scale)
{
  return { cv::Mat(map_size - template_size + cv::Size(1, 1), CV_64F, cv::Scalar(0)), cv::Mat(), scale };
}
template <>
histogram_t generateInitialDistribution<Hawkeye_WeightedShiftMode>(cv::Size template_size, cv::Size map_size,
                                                                   scale_t scale)
{
  return { cv::Mat(map_size - template_size + cv::Size(1, 1), CV_64F, cv::Scalar(0)),
           cv::Mat(map_size - template_size + cv::Size(1, 1), CV_64F, cv::Scalar(0)), scale };
}

double getSigma(const tf2::Transform& shift, double error_coeff, double scale)
{
  return std::hypot(shift.getOrigin().x(), shift.getOrigin().y()) * error_coeff / scale;
}

template <typename HawkeyeMode>
histogram_t generatePrior_(const histogram_t before, size_t kernel_size, double sigma)
{
  auto& [hist_mat, weight_mat, scale] = before;
  cv::Mat ret_h(hist_mat.size(), hist_mat.type());
  cv::GaussianBlur(hist_mat, ret_h, cv::Size(kernel_size, kernel_size), sigma, cv::BorderTypes::BORDER_REPLICATE);
  if constexpr (HawkeyeMode::is_weighted_)
  {
    cv::Mat ret_w(weight_mat.size(), weight_mat.type());
    cv::GaussianBlur(weight_mat, ret_w, cv::Size(kernel_size, kernel_size), sigma, cv::BorderTypes::BORDER_REPLICATE);
    return { ret_h, ret_w, scale };
  }
  else
  {
    return { ret_h, weight_mat, scale };
  }
}

template <typename HawkeyeMode>
histogram_t generatePrior(const histogram_t before, const tf2::Transform& shift, size_t kernel_size, double error_coeff)
{
  return generatePrior_<HawkeyeMode>(before, kernel_size, getSigma(shift, error_coeff, std::get<2>(before)));
}
template histogram_t generatePrior<Hawkeye_CopyShiftMode>(const histogram_t before, const tf2::Transform& shift,
                                                          size_t kernel_size, double error_coeff);
template histogram_t generatePrior<Hawkeye_WeightedShiftMode>(const histogram_t before, const tf2::Transform& shift,
                                                              size_t kernel_size, double error_coeff);

template <typename HawkeyeMode>
histogram_t generatePrior(const histogram_t before, const tf2::Transform& shift, double error_coeff)
{
  double sigma = getSigma(shift, error_coeff, std::get<2>(before));
  size_t estimated_half = std::max(0., std::ceil((sigma - 0.5) / 0.3));  // inversed sigma calculation
  size_t kernel_size = std::clamp<size_t>(estimated_half, 1, 5) * 2 + 1;
  return generatePrior_<HawkeyeMode>(before, kernel_size, sigma);
}
template histogram_t generatePrior<Hawkeye_CopyShiftMode>(const histogram_t before, const tf2::Transform& shift,
                                                          double error_coeff);
template histogram_t generatePrior<Hawkeye_WeightedShiftMode>(const histogram_t before, const tf2::Transform& shift,
                                                              double error_coeff);

match_result_t matchTemplate(const cv::Mat map, scale_t map_scale, const template_image_t template_image, bool use_mask)
{
  if (std::get<1>(template_image) != map_scale)
  {
    std::cerr << ERROR_PREFIX
              << "the scale of the map image"
                 " and the scale of the template image"
                 " are different."
              << ERROR_SUFFIX << std::endl;
    exit(4);
  }
  cv::Mat ret_h;
  double ret_w;

  const cv::Mat& template_mat = std::get<0>(template_image);
  if (use_mask)
  {
    cv::Mat template_mask = template_mat > 0;
    cv::matchTemplate(map, template_mat, ret_h, CV_TM_CCORR_NORMED, template_mask);  // 32bit
    ret_w = 1;
  }
  else
  {
    cv::matchTemplate(map, template_mat, ret_h, CV_TM_CCORR_NORMED);  // 32bit
    ret_w = 1;
  }
  if (false)
  {
    double color_min, color_max;
    cv::minMaxIdx(ret_h, &color_min, &color_max);
    std::cout << color_min << '~' << color_max;
  }
  ret_h.convertTo(ret_h, CV_64F);
  return { ret_h, ret_w, map_scale };
}

template <typename HawkeyeMode>
histogram_t updateDistribution(const histogram_t prior, const match_result_t match_result, double diminish,
                               double weight, double negative)
{
  auto& [prior_h, prior_w, prior_scale] = prior;
  auto& [match_h, match_w, match_scale] = match_result;
  if (prior_scale != match_scale)
  {
    std::cerr << ERROR_PREFIX
              << "the scale of the prior distribution"
                 " and the scale of the match result"
                 " are different."
              << ERROR_SUFFIX << std::endl;
    exit(4);
  }
  cv::Mat ret_h = prior_h * diminish + weight * (match_h + negative);
  cv::threshold(ret_h, ret_h, 0, 0, cv::ThresholdTypes::THRESH_TOZERO);
  if constexpr (HawkeyeMode::is_weighted_)
  {
    cv::Mat ret_w = prior_w * diminish + weight * (match_w + negative);
    return { ret_h, ret_w, prior_scale };
  }
  else
  {
    return { ret_h, prior_w, prior_scale };
  }
}
template histogram_t updateDistribution<Hawkeye_CopyShiftMode>(const histogram_t prior,
                                                               const match_result_t match_result, double diminish,
                                                               double weight, double negative);
template histogram_t updateDistribution<Hawkeye_WeightedShiftMode>(const histogram_t prior,
                                                                   const match_result_t match_result, double diminish,
                                                                   double weight, double negative);

template <>
weighted_histogram_t getWeightHistogram<Hawkeye_CopyShiftMode>(const histogram_t histogram, double weight_coeff)
{
  auto& [hist, weight, scale] = histogram;
  return { hist, scale };
}
template <>
weighted_histogram_t getWeightHistogram<Hawkeye_WeightedShiftMode>(const histogram_t histogram, double weight_coeff)
{
  auto& [hist, weight, scale] = histogram;
  if (weight_coeff == 0)
  {
    return { hist, scale };
  }
  return { hist / ((1 - weight_coeff) + weight_coeff * weight), scale };
}

cv::Moments getMoments_(const cv::Mat histogram)
{
  cv::Mat thres_hist;
  {
    double color_min, color_max;
    cv::minMaxIdx(histogram, &color_min, &color_max);
    cv::threshold(histogram - (color_min + color_max) / 2, thres_hist, 0, 0, CV_THRESH_TOZERO);
  }
  return cv::moments(thres_hist);
}

cv::Moments getMoments(const weighted_histogram_t histogram)
{
  return getMoments_(std::get<0>(histogram));
}
cv::Moments getMoments(const match_result_t histogram)
{
  return getMoments_(std::get<0>(histogram));
}

tf2::Vector3 average_(const cv::Size size, double scale, const cv::Moments& moments)
{
  auto& m = moments;
  if (m.m00 == 0)
  {
    return { 0, 0, 0 };
  }
  double gx = m.m10 / m.m00 - (size.width - 1) * 0.5;
  double gy = m.m01 / m.m00 - (size.height - 1) * 0.5;
  return { gx * scale, -gy * scale, 0 };
}

tf2::Vector3 average(const weighted_histogram_t histogram, const cv::Moments& moments)
{
  auto& [hist, scale] = histogram;
  return average_(hist.size(), scale, moments);
}
tf2::Vector3 average(const match_result_t histogram, const cv::Moments& moments)
{
  auto& [hist, weight, scale] = histogram;
  return average_(hist.size(), scale, moments);
}

tf2::Vector3 updateOffset(const tf2::Vector3& prev, const tf2::Vector3& next, double coeff)
{
  return (1 - coeff) * prev + coeff * next;
}

template <typename HawkeyeMode>
std::tuple<histogram_t, tf2::Vector3, tf2::Vector3> shiftCenter(const histogram_t histogram, const tf2::Vector3& center,
                                                                const tf2::Vector3& offset, double threshold_rate,
                                                                bool edge_copy_shift)
{
  auto& [hist, weight, scale] = histogram;
  auto size = hist.size();
  double th_x = size.width * scale * 0.5 * threshold_rate;
  double th_y = size.height * scale * 0.5 * threshold_rate;
  double x = offset.getX();
  double y = offset.getY();
  if (std::abs(x) <= th_x && std::abs(y) <= th_y)
  {
    return { histogram, center, offset };
  }
  cv::Mat next_hist, next_weight;
  if constexpr (HawkeyeMode::is_weighted_)
  {
    next_hist = cv::Mat(size, hist.type(), cv::Scalar(0));
    next_weight = cv::Mat(size, hist.type(), cv::Scalar(0));
  }
  else
  {
    if (edge_copy_shift)
    {
      next_hist = hist.clone();
    }
    else
    {
      next_hist = cv::Mat(size, hist.type(), cv::Scalar(0));
    }
  }
  cv::Size rect_size = size;
  cv::Point rect_p_prev(0, 0), rect_p_next(0, 0);
  tf2::Vector3 center_next = center;
  tf2::Vector3 offset_next = offset;
  if (std::abs(x) > th_x)
  {
    rect_size.width -= 1;
    if (x < 0)
    {
      rect_p_next.x = 1;
      center_next.setX(center_next.getX() - scale);
      offset_next.setX(offset_next.getX() + scale);
    }
    else
    {
      rect_p_prev.x = 1;
      center_next.setX(center_next.getX() + scale);
      offset_next.setX(offset_next.getX() - scale);
    }
  }
  if (std::abs(y) > th_y)
  {
    rect_size.height -= 1;
    if (y < 0)
    {
      rect_p_prev.y = 1;
      center_next.setY(center_next.getY() - scale);
      offset_next.setY(offset_next.getY() + scale);
    }
    else
    {
      rect_p_next.y = 1;
      center_next.setY(center_next.getY() + scale);
      offset_next.setY(offset_next.getY() - scale);
    }
  }
  hist(cv::Rect(rect_p_prev, rect_size)).copyTo(next_hist(cv::Rect(rect_p_next, rect_size)));
  if constexpr (HawkeyeMode::is_weighted_)
  {
    weight(cv::Rect(rect_p_prev, rect_size)).copyTo(next_weight(cv::Rect(rect_p_next, rect_size)));
  }
  return { { next_hist, next_weight, scale }, center_next, offset_next };
}
template std::tuple<histogram_t, tf2::Vector3, tf2::Vector3>
shiftCenter<Hawkeye_CopyShiftMode>(const histogram_t histogram, const tf2::Vector3& center, const tf2::Vector3& offset,
                                   double threshold_rate, bool edge_copy_shift);
template std::tuple<histogram_t, tf2::Vector3, tf2::Vector3>
shiftCenter<Hawkeye_WeightedShiftMode>(const histogram_t histogram, const tf2::Vector3& center,
                                       const tf2::Vector3& offset, double threshold_rate, bool edge_copy_shift);

cv::Mat convertHistogram2CVU8C1(const cv::Mat histogram)
{
  double color_min, color_max;
  cv::minMaxIdx(histogram, &color_min, &color_max);

  double abs_max = std::max(std::abs(color_min), std::abs(color_max));
  if (abs_max == 0)
  {
    abs_max = 1;
  }
  cv::Mat ret;
  {
    // ret = histogram * 255 / std::max(color_max, 1.);
    ret = (histogram - color_min) * 255 / (color_max - color_min);
    // ret = histogram * 127 / abs_max + 128;
  }
  ret.convertTo(ret, CV_8U);
  return ret;
}
cv::Mat convertHistogram2CVU8C1(const weighted_histogram_t histogram)
{
  auto& [hist, scale] = histogram;
  return convertHistogram2CVU8C1(hist);
}
cv::Mat convertHistogram2CVU8C1(const match_result_t histogram)
{
  auto& [hist, weight, scale] = histogram;
  return convertHistogram2CVU8C1(hist);
}

void setHistogramMarkers(visualization_msgs::MarkerArray& ma, const weighted_histogram_t histogram,
                         const tf2::Transform& odometry, const std_msgs::Header& header, bool as_array)
{
  ma.markers.clear();
  auto& [image, scale] = histogram;
  double max_weight, min_weight;
  {
    cv::minMaxLoc(image, &min_weight, &max_weight);
    if (max_weight == min_weight)
    {
      min_weight = max_weight - 1;
    }
  }
  cv::Size size = image.size();
  auto f = [min_weight, max_weight, base_point = odometry.getOrigin(), size, scale = scale](double val, cv::Point at) {
    geometry_msgs::Point point_out;
    std_msgs::ColorRGBA color_out;
    double rank = (val - min_weight) / (max_weight - min_weight);
    // rank *= rank;
    color_out.r = std::clamp(2 * rank, 0., 1.);
    color_out.g = std::clamp(2 * (1 - rank), 0., 1.);
    if (val < (min_weight + max_weight) / 2)
    {
      color_out.r *= 0.5;
      color_out.g *= 0.5;
    }
    color_out.b = 0;
    color_out.a = 1;
    point_out.x = base_point.x() + (at.x - (size.width - 1) * 0.5) * scale;
    point_out.y = base_point.y() - (at.y - (size.height - 1) * 0.5) * scale;
    point_out.z = 0;
    return std::make_pair(point_out, color_out);
  };
  auto checker = [min_weight, max_weight](double val) { return true; };
  if (as_array)
  {
    ma.markers.reserve(size.width * size.height);

    visualization_msgs::Marker marker;
    {
      marker.header = header;
      marker.ns = "histogram";
      marker.action = visualization_msgs::Marker::MODIFY;
      marker.type = visualization_msgs::Marker::ARROW;

      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;

      marker.id = 0;
    }

    for (int w = 0; w < size.width; ++w)
    {
      for (int h = 0; h < size.height; ++h)
      {
        if (double val = image.at<double>(cv::Point(w, h)); checker(val))
        {
          std::tie(marker.pose.position, marker.color) = f(val, cv::Point(w, h));
          ma.markers.push_back(marker);
          ++marker.id;
        }
      }
    }
  }
  else
  {
    auto& marker = ma.markers.emplace_back();
    {
      marker.header = header;
      marker.ns = "histogram";
      marker.action = visualization_msgs::Marker::MODIFY;
      marker.type = visualization_msgs::Marker::POINTS;

      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 1;

      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;

      marker.id = 0;
      marker.points.reserve(size.width * size.height);
      marker.colors.reserve(size.width * size.height);
    }
    for (int w = 0; w < size.width; ++w)
    {
      for (int h = 0; h < size.height; ++h)
      {
        if (double val = image.at<double>(cv::Point(w, h)); checker(val))
        {
          auto [point, color] = f(val, cv::Point(w, h));
          marker.points.emplace_back(point);
          marker.colors.emplace_back(color);
        }
      }
    }
  }
  return;
}

}  // namespace hawkeye::hawkeye_base