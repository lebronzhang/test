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

#ifndef PCL_IO_REAL_SENSE_GRABBER_H
#define PCL_IO_REAL_SENSE_GRABBER_H

#include <pcl/io/grabber.h>
//#include <pcl/io/real_sense_common.h>//// file position
#include "real_sense_common.h"

namespace pcl
{
  namespace io
  {
  	template <typename T> class Buffer;

  	namespace real_sense
  	{
  	  class RealSenseDevice;
  	  class Common;
  	}
  }

  using namespace pcl::io::real_sense;

  class PCL_EXPORTS RealSenseGrabber : public Grabber
  {
    public:

      typedef
            void (sig_cb_real_sense_point_cloud)
              (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
          
      typedef
            void (sig_cb_real_sense_point_cloud_rgba)
              (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);

      /** Create a grabber for a RealSense device.
        *
        * The grabber "captures" the device, making it impossible for other
        * grabbers to interact with it. The device is "released" when the
        * grabber is destructed.
        *
        * This will throw pcl::io::IOException if there are no free devices
        * that match the supplied \a device_id.
        *
        * \param[in] device_id device identifier, which can be a serial number,
        * a zero-based index (with '#' prefix), or an empty string (to select
        * the first available device)
        * \param[in] mode desired framerate and stream resolution (see Mode).
        * If the default is supplied, then the mode closest to VGA at 30 Hz
        * will be chosen.
        * \param[in] strict if set to \c true, an exception will be thrown if
        * device does not support exactly the mode requsted. Otherwise the
        * closest available mode is selected. */
      RealSenseGrabber (const std::string& device_id = "", const Common::Mode& mode = Common::Mode (), bool strict = false);

      virtual
      ~RealSenseGrabber () throw ();

      virtual void
      start ();

      virtual void
      stop ();

      virtual bool
      isRunning () const;

      virtual std::string
      getName () const;

      virtual float
      getFramesPerSecond () const;

      /** Set the confidence threshold for depth data.
        *
        * Valid range is [0..15]. Discarded points will have their coordinates
        * set to NaNs). */
      void
      setConfidenceThreshold (unsigned int threshold);

      /** Enable temporal filtering of the depth data received from the device.
        *
        * The window size parameter is not relevant for `RealSense_None`
        * filtering type.
        *
        * \note if the grabber is running and the new parameters are different
        * from the current parameters, grabber will be restarted. */
      void
      enableTemporalFiltering (Common::TemporalFilteringType type, size_t window_size);

      /** Disable temporal filtering. */
      void
      disableTemporalFiltering ();

      /** Get the serial number of device captured by the grabber. */
      const std::string&
      getDeviceSerialNumber () const;

      /** Get a list of capturing modes supported by the PXC device
        * controlled by this grabber.
        *
        * \param[in] only_depth list depth-only modes
        *
        * \note: this list exclude modes where framerates of the depth and
        * color streams do not match. */
      std::vector<Common::Mode>
      getAvailableModes (bool only_depth = false) const;

      /** Set desired capturing mode.
        *
        * \note if the grabber is running and the new mode is different the
        * one requested previously, grabber will be restarted. */
      void
      setMode (const Common::Mode& mode, bool strict = false);

      /** Get currently active capturing mode.
        *
        * \note: capturing mode is selected when start() is called; output of
        * this function before grabber was started is undefined. */
      const Common::Mode&
      getMode () const;

    private:

      // Signals to indicate whether new clouds are available
      boost::signals2::signal<sig_cb_real_sense_point_cloud>* point_cloud_signal_;
      boost::signals2::signal<sig_cb_real_sense_point_cloud_rgba>* point_cloud_rgba_signal_;

      void
      run ();

      void
      createDepthBuffer ();

      void
      selectMode ();

      Common common;
  };
}

#endif /* PCL_IO_REAL_SENSE_GRABBER_H */
