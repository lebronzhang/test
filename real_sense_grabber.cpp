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

#include <boost/lexical_cast.hpp>

#include <pcl/common/io.h>
#include <pcl/common/time.h>

#include <pcl/io/buffers.h>
#include <pcl/io/io_exception.h>
//#include <pcl/io/real_sense/real_sense_common.h>
#include "real_sense_grabber.h"
#include "real_sense_grabber_impl.h"
#include "real_sense_device_manager.h"

/*#ifdef USE_RSSDK
#include <pcl/io/real_sense/RSSDK/real_sense_device_manager.h>
#else
#include <pcl/io/real_sense/librealsense/real_sense_device_manager.h>
#endif*/

using namespace pcl::io;
using namespace pcl::io::real_sense;

pcl::RealSenseGrabber::Mode::Mode ()
: fps (0), depth_width (0), depth_height (0), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f)
: fps (f), depth_width (0), depth_height (0), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int dw, unsigned int dh)
: fps (0), depth_width (dw), depth_height (dh), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f, unsigned int dw, unsigned int dh)
: fps (f), depth_width (dw), depth_height (dh), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int dw, unsigned int dh, unsigned int cw, unsigned int ch)
: fps (0), depth_width (dw), depth_height (dh), color_width (cw), color_height (ch)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f, unsigned int dw, unsigned int dh, unsigned int cw, unsigned int ch)
: fps (f), depth_width (dw), depth_height (dh), color_width (cw), color_height (ch)
{
}

bool
pcl::RealSenseGrabber::Mode::operator== (const pcl::RealSenseGrabber::Mode& m) const
{
  return (this->fps == m.fps &&
  	      this->depth_width == m.depth_width &&
          this->depth_height == m.depth_height &&
          this->color_width == m.color_width &&
          this->color_height == m.color_height);
}

pcl::RealSenseGrabber::RealSenseGrabber (const std::string& device_id, const Mode& mode, bool strict)
: Grabber ()
, p_ (new pcl::io::real_sense::RealSenseGrabberImpl (this))
, is_running_ (false)
, confidence_threshold_ (6)
, temporal_filtering_type_ (RealSense_None)
, temporal_filtering_window_size_ (1)
, mode_requested_ (mode)
, strict_ (strict)
{
  if (device_id == "")
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice ();
  else if (device_id[0] == '#')
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (device_id);

  point_cloud_signal_ = createSignal<sig_cb_real_sense_point_cloud> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_real_sense_point_cloud_rgba> ();
}

pcl::RealSenseGrabber::~RealSenseGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_real_sense_point_cloud> ();
  disconnect_all_slots<sig_cb_real_sense_point_cloud_rgba> ();
}

void
pcl::RealSenseGrabber::start ()
{
  if (!is_running_)
  {
    need_xyz_ = num_slots<sig_cb_real_sense_point_cloud> () > 0;
    need_xyzrgba_ = num_slots<sig_cb_real_sense_point_cloud_rgba> () > 0;
    selectMode ();
    p_->start ();
  }
}

void
pcl::RealSenseGrabber::stop ()
{
  if (is_running_)
  {
    is_running_ = false;
    thread_.join ();
  }
}

bool
pcl::RealSenseGrabber::isRunning () const
{
  return (is_running_);
}

float
pcl::RealSenseGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

void
pcl::RealSenseGrabber::setConfidenceThreshold (unsigned int threshold)
{
  if (threshold > 15)
  {
    PCL_WARN ("[pcl::RealSenseGrabber::setConfidenceThreshold] Attempted to set threshold outside valid range (0-15)");
  }
  else
  {
    p_->setConfidenceThreshold (threshold);
  }
}

void
pcl::RealSenseGrabber::enableTemporalFiltering (TemporalFilteringType type, size_t window_size)
{
  if (temporal_filtering_type_ != type ||
     (type != RealSense_None && temporal_filtering_window_size_ != window_size))
  {
    temporal_filtering_type_ = type;
    temporal_filtering_window_size_ = window_size;
    if (is_running_)
    {
      stop ();
      start ();
    }
  }
}

void
pcl::RealSenseGrabber::disableTemporalFiltering ()
{
  enableTemporalFiltering (RealSense_None, 1);
}

const std::string&
pcl::RealSenseGrabber::getDeviceSerialNumber () const
{
  return (device_->getSerialNumber ());
}

std::vector<pcl::RealSenseGrabber::Mode>
pcl::RealSenseGrabber::getAvailableModes (bool only_depth) const
{
  return (p_->getAvailableModes (only_depth));
}

void
pcl::RealSenseGrabber::setMode (const Mode& mode, bool strict)
{
  if (mode == mode_requested_ && strict == strict_)
    return;
  mode_requested_ = mode;
  strict_ = strict;
  if (is_running_)
  {
    stop ();
    start ();
  }
}

float
pcl::RealSenseGrabber::computeModeScore (const Mode& mode)
{
  const float FPS_WEIGHT = 100000;
  const float DEPTH_WEIGHT = 1000;
  const float COLOR_WEIGHT = 1;
  int f = mode.fps - mode_requested_.fps;
  int dw = mode.depth_width - mode_requested_.depth_width;
  int dh = mode.depth_height - mode_requested_.depth_height;
  int cw = mode.color_width - mode_requested_.color_width;
  int ch = mode.color_height - mode_requested_.color_height;
  float penalty;
  penalty  = std::abs (FPS_WEIGHT * f * (mode_requested_.fps != 0));
  penalty += std::abs (DEPTH_WEIGHT * dw * (mode_requested_.depth_width != 0));
  penalty += std::abs (DEPTH_WEIGHT * dh * (mode_requested_.depth_height != 0));
  penalty += std::abs (COLOR_WEIGHT * cw * (mode_requested_.color_width != 0 && need_xyzrgba_));
  penalty += std::abs (COLOR_WEIGHT * ch * (mode_requested_.color_height != 0 && need_xyzrgba_));
  return penalty;
}

void
pcl::RealSenseGrabber::selectMode ()
{
  
  if (mode_requested_ == Mode ()) printf("mode_requested_ == Mode\n");
  else printf("mode_requested_ != Mode\n");
  if (mode_requested_ == Mode ())
    mode_requested_ = Mode (30, 640, 480, 640, 480);
  printf("mode_requested_ fps: %i\n",mode_requested_.fps);
  /*float best_score = std::numeric_limits<float>::max ();
  std::vector<Mode> modes = getAvailableModes (!need_xyzrgba_);
  for (size_t i = 0; i < modes.size (); ++i)
  {
    Mode mode = modes[i];
    float score = computeModeScore (mode);
    if (score < best_score)
    {
      best_score = score;
      mode_selected_ = mode;
    }
  }
  if (strict_ && best_score > 0)
    THROW_IO_EXCEPTION ("RealSense device does not support requested mode");*/
}

void
pcl::RealSenseGrabber::createDepthBuffer ()
{
  size_t size = mode_selected_.depth_width * mode_selected_.depth_height;
  switch (temporal_filtering_type_)
  {
  case RealSense_None:
  {
    depth_buffer_.reset (new pcl::io::SingleBuffer<unsigned short> (size));
    break;
  }
  case RealSense_Median:
  {
    depth_buffer_.reset (new pcl::io::MedianBuffer<unsigned short> (size, temporal_filtering_window_size_));
    break;
  }
  case RealSense_Average:
  {
    depth_buffer_.reset (new pcl::io::AverageBuffer<unsigned short> (size, temporal_filtering_window_size_));
    break;
  }
  }
}