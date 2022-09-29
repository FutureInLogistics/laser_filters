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

#ifndef LASER_SCAN_GHOST_NOISE_FILTER_H
#define LASER_SCAN_GHOST_NOISE_FILTER_H
/**
\author Dinko Osmankovic
@b GhostNoiseFilter description ghoes here

**/

#include "filters/filter_base.hpp"

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
    for(size_t i = 0; i < scan_size; ++i)
    {
      for(size_t j = 0; j < laser_scans_buffer.size(); ++j)
      {
        float val = laser_scans_buffer[j].ranges[i] / laser_scans_buffer.size();
        ranges_means[i] += val;
      }
    }
        

    for(size_t i = 0; i < scan_size; ++i)
    {
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

  double var_threshold_;
  double range_threshold_;
  double intensity_threshold_;
  double intensity_range_treshold_;
  int spatial_window_;
  double spatial_threshold_;
  int memory_buffer_size_;
  int scan_size;
  int k_neigbors_;
  std::vector<sensor_msgs::msg::LaserScan> laser_scans_buffer;  
  std::vector<double> ranges_means;
  std::vector<double> ranges_variance;
  //std::ofstream file;

  bool configure()
  {
    var_threshold_ = 10.0;
    range_threshold_ = 0.05;
    memory_buffer_size_ = 5;
    intensity_threshold_ = 50.0;
    intensity_range_treshold_ = 0.5;
    spatial_window_ = 5;
    spatial_threshold_ = 0.1;
    scan_size = 1368;
    k_neigbors_ = 5;
    getParam("var_threshold", var_threshold_);
    getParam("range_threshold", range_threshold_);
    getParam("memory_buffer_size", memory_buffer_size_);
    getParam("intensity_threshold", intensity_threshold_);
    getParam("intensity_range_treshold", intensity_range_treshold_);
    getParam("spatial_window", spatial_window_);
    getParam("spatial_threshold", spatial_threshold_);
    getParam("k_neigbors", k_neigbors_);
    ranges_means.resize(scan_size);
    ranges_variance.resize(scan_size);
    //file.open("debug.csv");
    return true;
  }

  virtual ~GhostNoiseFilter(){ /*file.close();*/ }

  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan)
  {    
    filtered_scan = input_scan;
    // add scan to memory buffer
    laser_scans_buffer.emplace_back(input_scan);
    if (laser_scans_buffer.size() > memory_buffer_size_)
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
      if (    (input_scan.ranges[i] < range_threshold_) || 
              (ranges_variance[i] > var_threshold_) ||
              ((input_scan.ranges[i] < intensity_range_treshold_) && input_scan.intensities[i] < intensity_threshold_)
         ) 
      {
        filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }

      // SPATIAL CONSISTENCY
      double r1 = filtered_scan.ranges[i];
      double t1 = angle_min + double(i) * angle_increment;
      int number_of_points_in_radii = 0;
      double dist = 0;
    
      //if (r1 > input_scan.range_max)
      //  continue;
      
      for (int j = -spatial_window_; j <= spatial_window_; ++j)
      {

        if (j == 0 || i < spatial_window_ || i > input_scan.ranges.size() - spatial_window_)
          continue;
        
        double r2 = filtered_scan.ranges[i+j];
        double t2 = angle_min + double(i+j) * angle_increment;

        dist = sqrt(  r1*r1 + r2*r2 - 2*r1*r2*cosf(t2-t1)  );

        if ( dist < spatial_threshold_ ) 
        {
          number_of_points_in_radii++;
        }
      }
      //printf("Point (%.4f, %.4f) \tN : %d\n", r1, t1, number_of_points_in_radii);
      if (number_of_points_in_radii < k_neigbors_)
        filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();  
        
    }

    // clear means and variances
    for(size_t i = 0; i < scan_size; ++i)
    {
      ranges_means[i] = 0.0;
      ranges_variance[i] = 0.0;
    }

    return true;
  }
};
}

#endif // LASER_SCAN_GHOST_NOISE_FILTER_H
