
/***************************************************************************
 *  urg_aqt.cpp - Thread to retrieve laser data from Hokuyo URG
 *
 *  Created: Sat Nov 28 01:31:26 2009
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "urg_aqt.h"

#include <core/threading/mutex.h>
#include <utils/time/wait.h>

#include <urg/UrgCtrl.h>
#include <urg/RangeSensorParameter.h>

#include <memory>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>
#include <cerrno>
#include <sys/file.h>
#include <unistd.h>
#include <limits>
#ifdef HAVE_LIBUDEV
#  include <cstring>
#  ifdef __cplusplus
extern "C" {
#  endif
#  include <libudev.h>
#  ifdef __cplusplus
}
#  endif
#endif

using namespace qrk;
using namespace fawkes;



/** @class HokuyoUrgAcquisitionThread "urg_aqt.h"
 * Laser acqusition thread for Hokuyo URG laser range finders.
 * This thread fetches the data from the laser.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 */
HokuyoUrgAcquisitionThread::HokuyoUrgAcquisitionThread(std::string &cfg_name,
						       std::string &cfg_prefix)
  : LaserAcquisitionThread("HokuyoUrgAcquisitionThread")
{
  set_name("HokuyoURG(%s)", cfg_name.c_str());
  pre_init_done_ = false;
  cfg_name_   = cfg_name;
  cfg_prefix_ = cfg_prefix;
}


void
HokuyoUrgAcquisitionThread::pre_init(fawkes::Configuration *config,
				     fawkes::Logger        *logger)
{
  if (pre_init_done_)  return;

  number_of_values_ = _distances_size = 360;

  pre_init_done_ = true;
}

void
HokuyoUrgAcquisitionThread::init()
{
  pre_init(config, logger);

#ifdef HAVE_LIBUDEV
  try {
    cfg_device_ = config->get_string((cfg_prefix_ + "device").c_str());
  } catch (Exception &e) {
    // check if bus/port numbers are given
    try {
      cfg_device_ = "";
      cfg_serial_ = config->get_string((cfg_prefix_ + "serial").c_str());

      // try to find device using udev
      struct udev *udev;
      struct udev_enumerate *enumerate;
      struct udev_list_entry *devices, *dev_list_entry;
      struct udev_device *dev, *usb_device;
      udev = udev_new();
      if (!udev) {
	throw Exception("HokuyoURG: Failed to initialize udev for "
			"device detection");
      }

      enumerate = udev_enumerate_new(udev);
      udev_enumerate_add_match_subsystem(enumerate, "tty");
      udev_enumerate_scan_devices(enumerate);

      devices = udev_enumerate_get_list_entry(enumerate);
      udev_list_entry_foreach(dev_list_entry, devices) {

	const char *path;

	path = udev_list_entry_get_name(dev_list_entry);
	dev = udev_device_new_from_syspath(udev, path);

	usb_device = udev_device_get_parent_with_subsystem_devtype(dev, "usb",
								   "usb_device");
	if (! dev || ! usb_device) continue;
	
	if ( (strcmp(udev_device_get_sysattr_value(usb_device,"manufacturer"),
		     "Hokuyo Data Flex for USB") == 0) &&
	     (strcmp(udev_device_get_sysattr_value(usb_device,"product"),
		     "URG-Series USB Driver") == 0) )
	{

	  const char *devpath = udev_device_get_devnode(dev);
	  int urgfd = open(devpath, 0, O_RDONLY);
          if (urgfd == -1) {
            logger->log_info(name(), "Failed to probe %s, cannot open file: %s",
			     devpath, strerror(errno));
	    continue;
	  }
          if (flock(urgfd, LOCK_EX | LOCK_NB) != 0) {
            logger->log_info(name(), "Failed to probe %s, cannot lock file: %s",
			     devpath, strerror(errno));
	    close(urgfd);
	    continue;
	  }
	  UrgCtrl probe_ctrl;
	  if ( ! probe_ctrl.connect(devpath) )  {
	    logger->log_info(name(), "Failed to probe %s: %s", devpath,
			     probe_ctrl.what());
	    flock(urgfd, LOCK_UN);
	    close(urgfd);
	    continue;
	  }

	  std::map<std::string, std::string> devinfo;
	  try {
	    devinfo = get_device_info(&probe_ctrl);
	  } catch (Exception &e) {
	    logger->log_info(name(), "Failed to probe device info %s: %s",
			     devpath, e.what());
            flock(urgfd, LOCK_UN);
	    close(urgfd);
	    continue;
	  }
          flock(urgfd, LOCK_UN);
	  close(urgfd);

	  if (devinfo["SERI"] == cfg_serial_) {
	    cfg_device_ = devpath;

	    logger->log_info(
	      name(), "Matching URG at %s (vendor: %s (%s), "
	      "product: %s (%s), serial %s)", devpath,
	      udev_device_get_sysattr_value(usb_device, "manufacturer"),
	      udev_device_get_sysattr_value(usb_device, "idVendor"),
	      udev_device_get_sysattr_value(usb_device, "product"),
	      udev_device_get_sysattr_value(usb_device, "idProduct"),
	      devinfo["SERI"].c_str());

	    break;
	  } else {
	    logger->log_info(name(), "Non-matching URG with serial %s at %s",
			     devinfo["SERI"].c_str(), devpath);
	  }
	}
      }
      udev_enumerate_unref(enumerate);
      udev_unref(udev);

      if (cfg_device_ == "") {
	throw Exception("No Hokuyo URG with serial %s found",
			cfg_serial_.c_str());
      }

    } catch (Exception &e2) {
      e.append(e2);
      throw e;
    }
  }
#else
  cfg_device_ = config->get_string((cfg_prefix_ + "device").c_str());
#endif

  ctrl_ = new UrgCtrl();
#if __cplusplus >= 201103L
  std::unique_ptr<UrgCtrl> ctrl(ctrl_);
#else
  std::auto_ptr<UrgCtrl> ctrl(ctrl_);
#endif
  fd_ = open(cfg_device_.c_str(), 0, O_RDONLY);
  if (fd_ == -1) {
    throw Exception(errno, "Failed to open URG device %s", cfg_device_.c_str());
  }
  if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    close(fd_);
    throw Exception("Failed to acquire lock for URG device %s", cfg_device_.c_str());
  }
  if ( ! ctrl_->connect(cfg_device_.c_str()) ) {
    close(fd_);
    flock(fd_, LOCK_UN);
    throw Exception("Connecting to URG laser failed: %s", ctrl_->what());
  }

  ctrl_->setCaptureMode(AutoCapture);
  device_info_ = get_device_info(ctrl_);

  if (device_info_.find("PROD") == device_info_.end()) {
    close(fd_);
    flock(fd_, LOCK_UN);
    throw Exception("Failed to read product info for URG laser");
  }

  logger->log_info(name(), "Using device file %s", cfg_device_.c_str());
  std::map<std::string, std::string>::iterator di;
  for (di = device_info_.begin(); di != device_info_.end(); ++di) {
    logger->log_info(name(), "%s: %s", di->first.c_str(), di->second.c_str());
  }

  scan_msec_ = ctrl_->scanMsec();
  float distance_min = 0.;
  float distance_max = 0.;

  try {
    first_ray_     = config->get_uint((cfg_prefix_ + "first_ray").c_str());
    last_ray_      = config->get_uint((cfg_prefix_ + "last_ray").c_str());
    front_ray_     = config->get_uint((cfg_prefix_ + "front_ray").c_str());
    slit_division_ = config->get_uint((cfg_prefix_ + "slit_division").c_str());
  } catch (Exception &e) {
    logger->log_info(name(), "No or incomplete config data, reading from device");
    // Get data from device
    RangeSensorParameter p = ctrl_->parameter();
    first_ray_     = p.area_min;
    last_ray_      = p.area_max;
    front_ray_     = p.area_front;
    slit_division_ = p.area_total;
    distance_min    = p.distance_min / 1000.;
    distance_max    = p.distance_max / 1000.;
  }

  step_per_angle_ = slit_division_ / 360.;
  angle_per_step_ = 360. / slit_division_;
  angular_range_  = (last_ray_ - first_ray_) * angle_per_step_;

  logger->log_info(name(), "Time per scan: %li msec", scan_msec_);
  logger->log_info(name(), "Rays range:    %u..%u, front at %u",
		   first_ray_, last_ray_, front_ray_);
  logger->log_info(name(), "Slit Division: %u", slit_division_);
  logger->log_info(name(), "Step/Angle:    %f", step_per_angle_);
  logger->log_info(name(), "Angle/Step:    %f deg", angle_per_step_);
  logger->log_info(name(), "Angular Range: %f deg", angular_range_);
  logger->log_info(name(), "Min dist:      %f m", distance_min);
  logger->log_info(name(), "Max dist:      %f m", distance_max);


  cfg_time_offset_ = 0.;
  try {
    float time_factor =
      config->get_float((cfg_prefix_ + "time_offset_scan_time_factor").c_str());
    cfg_time_offset_ = (scan_msec_ / -1000.) * time_factor;
  } catch (Exception &e) {} // ignored, use default

  try {
    cfg_time_offset_ += config->get_float((cfg_prefix_ + "time_offset").c_str());
  } catch (Exception &e) {} // ignored, use default

  // that should be 1000 really to convert msec -> usec. But empirically
  // the results are slightly better with 990 as factor.
  timer_ = new TimeWait(clock, scan_msec_ * 990);

  alloc_distances(number_of_values_);

  ctrl.release();
}


void
HokuyoUrgAcquisitionThread::finalize()
{
  free(_distances);
  _distances = NULL;
  delete timer_;

  ctrl_->stop();
  delete ctrl_;

  close(fd_);
  flock(fd_, LOCK_UN);

  logger->log_debug(name(), "Stopping laser");
}


void
HokuyoUrgAcquisitionThread::loop()
{
  timer_->mark_start();

  std::vector<long> values;
  int num_values = ctrl_->capture(values);
  if (num_values > 0) {
    //logger->log_debug(name(), "Captured %i values", num_values);
    _data_mutex->lock();

    _new_data = true;
    _timestamp->stamp();
    *_timestamp += cfg_time_offset_;
    for (unsigned int a = 0; a < 360; ++a) {
      unsigned int front_idx = front_ray_ + roundf(a * step_per_angle_);
      unsigned int idx = front_idx % slit_division_;
      if ( (idx >= first_ray_) && (idx <= last_ray_) ) {
        switch (values[idx]) // See the SCIP2.0 reference on page 12, Table 3
        {
        case 0: // Detected object is possibly at 22m
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 1: // Reflected light has low intensity
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 2: // Reflected light has low intensity
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 6: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 7: // Distance data on the preceding and succeeding steps have errors
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 8: // Intensity difference of two waves
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 9: // The same step had error in the last two scan
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 10: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 11: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 12: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 13: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 14: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 15: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 16: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 17: // Others
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 18: // Error reading due to strong reflective object
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        case 19: // Non-Measurable step
          _distances[a] = std::numeric_limits<float>::quiet_NaN();
          break;
        default:
          // div by 1000.f: mm -> m
          _distances[a] = values[idx] / 1000.f;
        }
      }
    }
    _data_mutex->unlock();
  //} else {
    //logger->log_warn(name(), "No new scan available, ignoring");
  }

  timer_->wait();
}

std::map<std::string, std::string>
  HokuyoUrgAcquisitionThread::get_device_info(qrk::UrgCtrl *ctrl)
{
  std::map<std::string, std::string> device_info;

  std::vector<std::string> version_info;
  if (ctrl->versionLines(version_info)) {
    for (unsigned int i = 0; i < version_info.size(); ++i) {
      std::string::size_type colon_idx      = version_info[i].find(":");
      std::string::size_type semi_colon_idx = version_info[i].find(";");
      if ((colon_idx == std::string::npos) ||
	  (semi_colon_idx == std::string::npos)) {
	logger->log_warn(name(), "Could not understand version info string '%s'",
			 version_info[i].c_str());
      } else {
	std::string::size_type val_len = semi_colon_idx - colon_idx - 1;
	std::string key   = version_info[i].substr(0, colon_idx);
	std::string value = version_info[i].substr(colon_idx+1, val_len);
	device_info[key] = value;
      }
    }
  } else {
    throw Exception("Failed retrieving version info: %s", ctrl->what());
  }
  return device_info;
}
