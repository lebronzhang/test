/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Copyright (c) 2016, Intel Corporation
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */
#include <pcl/common/io.h>
#include <pcl/io/buffers.h>
//#include <pcl/io/io_exception.h>
//#include <pcl/io/real_sense/librealsense/real_sense_grabber.h>

#include "real_sense_grabber_impl.h"
#include "real_sense_device_manager.h"

#include <librealsense/rs.hpp>

using namespace pcl::io;
using namespace pcl::io::real_sense;

pcl::io::real_sense::RealSenseGrabberImpl::RealSenseGrabberImpl (RealSenseGrabber* parent)
: p_ (parent)
{
}

pcl::io::real_sense::RealSenseGrabberImpl::~RealSenseGrabberImpl () throw ()
{
}

void
pcl::io::real_sense::RealSenseGrabberImpl::start ()
{
  /*if (p->need_xyz_ || p->need_xyzrgba_)
  {
    //select mode

    //initial depth parameters
  }
  if (p->need_xyzrgba_)
  {
    //initial color parameters
  }*/
  //initial the device
  p_->frequency_.reset ();
  p_->is_running_ = true;
  p_->thread_ = boost::thread (&RealSenseGrabberImpl::run, this);
}

void
pcl::io::real_sense::RealSenseGrabberImpl::setConfidenceThreshold (unsigned int threshold)
{
  //fix
}

std::vector<pcl::RealSenseGrabber::Mode>
pcl::io::real_sense::RealSenseGrabberImpl::getAvailableModes (bool only_depth) const
{
  std::vector<pcl::RealSenseGrabber::Mode> modes;

  return modes;
}

void
pcl::io::real_sense::RealSenseGrabberImpl::run ()
{
  /*const int WIDTH = mode_selected_.depth_width;
  const int HEIGHT = mode_selected_.depth_height;
  const int SIZE = WIDTH * HEIGHT;*/
  rs::device* device = p_->device_->getDevice ();
  device->enable_stream (rs::stream::depth, rs::preset::best_quality);
  device->enable_stream (rs::stream::color, rs::preset::best_quality);
  device->start ();
  int count = 0;
  int pos=0;
  int depth_width_;
  int depth_height_;
  int depth_size_;
  int color_width_;
  int color_height_;
  int color_size_;
  ///Buffer to store depth data
  std::vector<uint16_t> depth_data_;
  ///Buffer to store color data
  std::vector<uint8_t> color_data_;
  while (p_->is_running_)
  {
    if (device->is_streaming ()) device->wait_for_frames ();
    p_->fps_mutex_.lock ();
    p_->frequency_.event ();
    p_->fps_mutex_.unlock ();
    // Retrieve our images
    const uint16_t * depth_image = (const uint16_t *)device->get_frame_data (rs::stream::depth);
    const uint8_t * color_image = (const uint8_t *)device->get_frame_data (rs::stream::color);
    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = device->get_stream_intrinsics (rs::stream::depth);
    rs::intrinsics color_intrin = device->get_stream_intrinsics (rs::stream::color);
    rs::extrinsics depth_to_color = device->get_extrinsics (rs::stream::depth, rs::stream::color);
    float scale = device->get_depth_scale ();
    
    if (count == 0)
    {
      depth_width_ = depth_intrin.width;
      depth_height_ = depth_intrin.height;
      depth_size_ = depth_width_ * depth_height_;
      color_width_ = color_intrin.width;
      color_height_ = color_intrin.height;
      color_size_ = color_width_ * color_height_ * 3;
      depth_data_.resize (depth_size_);
      color_data_.resize (color_size_);
      //std::cerr<<"depth_width: "<<depth_width_<<std::endl;
    }
    
    depth_data_.clear ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;
    float max_distance = 6; // get rid of noisy data that is past 6 meters
    static const float nan = std::numeric_limits<float>::quiet_NaN ();

    memcpy (depth_data_.data (), &depth_image[0], depth_size_ * sizeof (uint16_t));

    if (p_->need_xyzrgba_)
    {
      color_data_.clear ();
      memcpy (color_data_.data (), &color_image[0], color_size_ * sizeof (uint8_t));
      xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (depth_width_, depth_height_));
      xyzrgba_cloud->is_dense = false;
      if (p_->need_xyz_)
      {
        xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (depth_width_, depth_height_));
        xyz_cloud->is_dense = false;
      }
      for (int dy = 0; dy < depth_height_; ++dy)
      {
        uint i = dy * depth_width_ - 1;
        for (int dx = 0; dx < depth_width_; ++dx)
        {
          i++;
          // Retrieve the 16-bit depth value and map it into a depth in meters
          uint16_t depth_value = depth_data_[i];
          float depth_in_meters = depth_value * scale;

          // Map from pixel coordinates in the depth image to real world co-ordinates
          rs::float2 depth_pixel = {(float)dx, (float)dy};
          rs::float3 depth_point = depth_intrin.deproject (depth_pixel, depth_in_meters);
          rs::float3 color_point = depth_to_color.transform (depth_point);
          rs::float2 color_pixel = color_intrin.project (color_point);

          const int cx = (int)std::round (color_pixel.x), cy = (int)std::round (color_pixel.y);
          int red = 0, green = 0, blue = 0;
          if (cx < 0 || cy < 0 || cx >= color_width_ || cy >= color_height_)
          {
            red = 255; green = 255; blue = 255;
          }
          else
          {
            int pos = (cy * color_width_ + cx) * 3;
            red =  color_data_[pos];
            green = color_data_[pos + 1];
            blue = color_data_[pos + 2];
          }
          if (depth_value == 0 || depth_point.z > max_distance)
          {
            xyzrgba_cloud->points[i].x = xyzrgba_cloud->points[i].y = xyzrgba_cloud->points[i].z = (float) nan;
            if (p_->need_xyz_)
            {
              xyz_cloud->points[i].x = xyz_cloud->points[i].y = xyz_cloud->points[i].z = (float) nan;
            }
            continue;
          }
          else
          {
            xyzrgba_cloud->points[i].x = depth_point.x;
            xyzrgba_cloud->points[i].y = -depth_point.y;
            xyzrgba_cloud->points[i].z = -depth_point.z;
            xyzrgba_cloud->points[i].r = red;
            xyzrgba_cloud->points[i].g = green;
            xyzrgba_cloud->points[i].b = blue;
            if (p_->need_xyz_)
            {
              xyz_cloud->points[i].x = depth_point.x;
              xyz_cloud->points[i].y = -depth_point.y;
              xyz_cloud->points[i].z = -depth_point.z;
            }
          }
        }
      }
      p_->point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
      if (p_->need_xyz_)
      {
        p_->point_cloud_signal_->operator () (xyz_cloud);
      }
    }
    else if (p_->need_xyz_)
    {
      xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (depth_width_, depth_height_));
      xyz_cloud->is_dense = false;
      for (int dy = 0; dy < depth_height_; ++dy)
      {
        uint i = dy * depth_width_ - 1;
        for (int dx = 0; dx < depth_width_; ++dx)
        {
          i++;
          // Retrieve the 16-bit depth value and map it into a depth in meters
          uint16_t depth_value = depth_data_[i];
          float depth_in_meters = depth_value * scale;
          rs::float2 depth_pixel = {(float)dx, (float)dy};
          rs::float3 depth_point = depth_intrin.deproject (depth_pixel, depth_in_meters);
          if (depth_value == 0 || depth_point.z > max_distance)
          {
            xyz_cloud->points[i].x = xyz_cloud->points[i].y = xyz_cloud->points[i].z = (float) nan;
            continue;
          }
          else
          {
            xyz_cloud->points[i].x = depth_point.x;
            xyz_cloud->points[i].y = -depth_point.y;
            xyz_cloud->points[i].z = -depth_point.z;
          }
        }
      }
      p_->point_cloud_signal_->operator () (xyz_cloud);
    }
    else
    {
      //do nothing
    }
    count++;
    if (count == 100000) count = 1;
  }
}