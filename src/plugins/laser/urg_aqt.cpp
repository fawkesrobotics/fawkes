
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
  __pre_init_done = false;
  __cfg_name   = cfg_name;
  __cfg_prefix = cfg_prefix;
}


void
HokuyoUrgAcquisitionThread::pre_init(fawkes::Configuration *config,
				     fawkes::Logger        *logger)
{
  if (__pre_init_done)  return;

  __number_of_values = _distances_size = 360;

  __pre_init_done = true;
}

void
HokuyoUrgAcquisitionThread::init()
{
  pre_init(config, logger);

#ifdef HAVE_LIBUDEV
  try {
    __cfg_device = config->get_string((__cfg_prefix + "device").c_str());
  } catch (Exception &e) {
    // check if bus/port numbers are given
    try {
      __cfg_device = "";
      __cfg_serial = config->get_string((__cfg_prefix + "serial").c_str());

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

	  if (devinfo["SERI"] == __cfg_serial) {
	    __cfg_device = devpath;

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

      if (__cfg_device == "") {
	throw Exception("No Hokuyo URG with serial %s found",
			__cfg_serial.c_str());
      }

    } catch (Exception &e2) {
      e.append(e2);
      throw e;
    }
  }
#else
  __cfg_device = config->get_string((__cfg_prefix + "device").c_str());
#endif

  __ctrl = new UrgCtrl();
  std::auto_ptr<UrgCtrl> ctrl(__ctrl);
  __fd = open(__cfg_device.c_str(), 0, O_RDONLY);
  if (__fd == -1) {
    throw Exception(errno, "Failed to open URG device %s", __cfg_device.c_str());
  }
  if (flock(__fd, LOCK_EX | LOCK_NB) != 0) {
    close(__fd);
    throw Exception("Failed to acquire lock for URG device %s", __cfg_device.c_str());
  }
  if ( ! __ctrl->connect(__cfg_device.c_str()) ) {
    close(__fd);
    flock(__fd, LOCK_UN);
    throw Exception("Connecting to URG laser failed: %s", __ctrl->what());
  }

  __ctrl->setCaptureMode(AutoCapture);
  __device_info = get_device_info(__ctrl);

  if (__device_info.find("PROD") == __device_info.end()) {
    close(__fd);
    flock(__fd, LOCK_UN);
    throw Exception("Failed to read product info for URG laser");
  }

  logger->log_info(name(), "Using device file %s", __cfg_device.c_str());
  std::map<std::string, std::string>::iterator di;
  for (di = __device_info.begin(); di != __device_info.end(); ++di) {
    logger->log_info(name(), "%s: %s", di->first.c_str(), di->second.c_str());
  }

  int scan_msec = __ctrl->scanMsec();
  float distance_min = 0.;
  float distance_max = 0.;

  try {
    __first_ray     = config->get_uint((__cfg_prefix + "first_ray").c_str());
    __last_ray      = config->get_uint((__cfg_prefix + "last_ray").c_str());
    __front_ray     = config->get_uint((__cfg_prefix + "front_ray").c_str());
    __slit_division = config->get_uint((__cfg_prefix + "slit_division").c_str());
  } catch (Exception &e) {
    logger->log_info(name(), "No or incomplete config data, reading from device");
    // Get data from device
    RangeSensorParameter p = __ctrl->parameter();
    __first_ray     = p.area_min;
    __last_ray      = p.area_max;
    __front_ray     = p.area_front;
    __slit_division = p.area_total;
    distance_min    = p.distance_min / 1000.;
    distance_max    = p.distance_max / 1000.;
  }

  __step_per_angle = __slit_division / 360.;
  __angle_per_step = 360. / __slit_division;
  __angular_range  = (__last_ray - __first_ray) * __angle_per_step;

  logger->log_info(name(), "Time per scan: %i msec", scan_msec);
  logger->log_info(name(), "Rays range:    %u..%u, front at %u",
		   __first_ray, __last_ray, __front_ray);
  logger->log_info(name(), "Slit Division: %u", __slit_division);
  logger->log_info(name(), "Step/Angle:    %f", __step_per_angle);
  logger->log_info(name(), "Angle/Step:    %f deg", __angle_per_step);
  logger->log_info(name(), "Angular Range: %f deg", __angular_range);
  logger->log_info(name(), "Min dist:      %f m", distance_min);
  logger->log_info(name(), "Max dist:      %f m", distance_max);

  // that should be 1000 really to convert msec -> usec. But empirically
  // the results are slightly better with 990 as factor.
  __timer = new TimeWait(clock, scan_msec * 990);

  alloc_distances(__number_of_values);

  ctrl.release();
}


void
HokuyoUrgAcquisitionThread::finalize()
{
  free(_distances);
  _distances = NULL;
  delete __timer;

  __ctrl->stop();
  delete __ctrl;

  close(__fd);
  flock(__fd, LOCK_UN);

  logger->log_debug(name(), "Stopping laser");
}


void
HokuyoUrgAcquisitionThread::loop()
{
  __timer->mark_start();

  std::vector<long> values;
  int num_values = __ctrl->capture(values);
  if (num_values > 0) {
    //logger->log_debug(name(), "Captured %i values", num_values);
    _data_mutex->lock();

    _new_data = true;
    for (unsigned int a = 0; a < 360; ++a) {
      unsigned int front_idx = __front_ray + roundf(a * __step_per_angle);
      unsigned int idx = front_idx % __slit_division;
      if ( (idx >= __first_ray) && (idx <= __last_ray) ) {
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

  __timer->wait();
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
