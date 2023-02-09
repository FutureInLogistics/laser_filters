/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#pragma once

/**
\author Dinko Osmankovic
@b GhostNoiseFilter description ghoes here

**/

#include "filters/filter_base.hpp"

#include "ghost_noise_filter_parameters.hpp" // automatically generated

#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

// DEBUG
#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <cmath>

namespace laser_filters
{

class GhostNoiseFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
  private:
  void calculateMeanVariance()
  {
    // calculate means of ranges
    for(size_t i = 0; i < params.scan_size; ++i)
    {
      ranges_means[i] = 0.0;
      for(size_t j = 0; j < laser_scans_buffer.size(); ++j)
      {
        float val = laser_scans_buffer[j].ranges[i] / laser_scans_buffer.size();
        ranges_means[i] += val;
      }
    }

    for(size_t i = 0; i < params.scan_size; ++i)
    {
      ranges_variance[i] = 0.0;
      for(size_t j = 0; j < laser_scans_buffer.size(); ++j)
      {
        float val = (laser_scans_buffer[j].ranges[i] - ranges_means[i]) * (laser_scans_buffer[j].ranges[i] - ranges_means[i]) /
                                   laser_scans_buffer.size();

        ranges_variance[i] += val;
      }
    }
  }

  double pointDistance(double r1, double t1, double r2, double t2)
  {
    return sqrt(  r1*r1 + r2*r2 - 2*r1*r2*cosf(t2-t1)  );
  }

public:
  explicit GhostNoiseFilter()
    : filters::FilterBase<sensor_msgs::msg::LaserScan>()
  {
    //
  }

  virtual ~GhostNoiseFilter()
  {
    /*file.close();*/
  }

  std::unique_ptr<ghost_noise_filter::ParamListener> param_listener;
  ghost_noise_filter::Params params;

  std::vector<sensor_msgs::msg::LaserScan> laser_scans_buffer;
  std::vector<double> ranges_means;
  std::vector<double> ranges_variance;
  //std::ofstream file;

  bool configure() override
  {
    param_listener = std::make_unique<ghost_noise_filter::ParamListener>(params_interface_, param_prefix_);
    params = param_listener->get_params();

    ranges_means.resize(params.scan_size);
    ranges_variance.resize(params.scan_size);

    //file.open("debug.csv");
    return true;
  }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) override
  {
    // check if parameters changed, update if necessary
    if (param_listener->is_old(params)) {
      params = param_listener->get_params();
    }

    filtered_scan = input_scan;
    // add scan to memory buffer
    laser_scans_buffer.emplace_back(input_scan);
    if (laser_scans_buffer.size() > params.memory_buffer_size)
    {
      laser_scans_buffer.erase(laser_scans_buffer.begin());
    }

    // compute mean and var from the memory buffer
    calculateMeanVariance();

    double angle_min = input_scan.angle_min;
    double angle_max = input_scan.angle_max;
    double angle_increment = input_scan.angle_increment;


    //std::cout << "Max | Min: \t" << *std::max_element(ranges_variance.begin(), ranges_variance.end()) << " | " <<
    //                                *std::min_element(ranges_variance.begin(), ranges_variance.end()) << std::endl;

    // update the filtered scan
    for (size_t i = 0; i < input_scan.ranges.size(); ++i)
    {
      if (    (input_scan.ranges[i] < params.range_threshold) ||
              (ranges_variance[i] > params.var_threshold) ||
              ((input_scan.ranges[i] < params.intensity_range_treshold) && input_scan.intensities[i] < params.intensity_threshold)
         )
      {
        filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }

      {
        // MIN ANGLE between Neighbours
        // 1. Get the angle to both neighbours
        // 2. Remove points with low angle
        const float this_point = input_scan.ranges[i];

        const float previous = (i == 0)
                             ? this_point
                             : input_scan.ranges[i-1];

        const float next = (i >= input_scan.ranges.size()-1)
                         ? this_point
                         : input_scan.ranges[i+1];

        // distance to the neighbour ray
        const float dist_to_neighbour = std::abs(this_point * std::sin(angle_increment));

        const float radial_dist_1 = std::abs(this_point - previous);
        const float radial_dist_2 = std::abs(next - this_point);

        const float angle_1 = std::atan2(dist_to_neighbour, radial_dist_1);
        const float angle_2 = std::atan2(dist_to_neighbour, radial_dist_2);
        const float angle = (angle_1 + angle_2) / 2.f;

        if (angle < params.min_angle_threshold)
        {
          filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
      }

      // SPATIAL CONSISTENCY
      double r1 = filtered_scan.ranges[i];
      double t1 = angle_min + double(i) * angle_increment;
      int number_of_points_in_radii = 0;
      double dist = 0;

      //if (r1 > input_scan.range_max)
      //  continue;

      for (int j = -params.spatial_window; j <= params.spatial_window; ++j)
      {
        if (j == 0 || i < params.spatial_window || i > input_scan.ranges.size() - params.spatial_window)
        {
          continue;
        }

        double r2 = filtered_scan.ranges[i+j];
        double t2 = angle_min + double(i+j) * angle_increment;

        dist = sqrt(  r1*r1 + r2*r2 - 2*r1*r2*cosf(t2-t1)  );

        if ( dist < params.spatial_threshold)
        {
          number_of_points_in_radii++;
        }
      }

      //printf("Point (%.4f, %.4f) \tN : %d\n", r1, t1, number_of_points_in_radii);
      if (number_of_points_in_radii < params.k_neigbors)
      {
        filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }

    }

    return true;
  }
};
}
