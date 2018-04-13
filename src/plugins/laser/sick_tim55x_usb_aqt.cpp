
/***************************************************************************
 *  sick_tim55x_aqt.cpp - Thread to retrieve laser data from Sick TiM55x
 *
 *  Created: Tue Jun 10 16:53:23 2014
 *  Copyright  2008-2014  Tim Niemueller [www.niemueller.de]
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

#include "sick_tim55x_usb_aqt.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/misc/string_split.h>
#include <utils/math/angle.h>

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <libusb.h>

#ifndef LIBUSB_API_VERSION
#  define libusb_error_name(error) ""
#  define LIBUSB_LOG_LEVEL_ERROR 1
#endif

#if LIBUSBX_API_VERSION < 0x01000102
   // libusb before 1.0.16 does not have libusb_strerror
#  define libusb_strerror libusb_error_name
#endif

using namespace fawkes;

#define USB_VENDOR  0x19A2
#define USB_PRODUCT 0x5001
#define USB_TIMEOUT 500


/** @class SickTiM55xUSBAcquisitionThread "sick_tim55x_usb_aqt.h"
 * Laser acqusition thread for Sick TiM55x laser range finders.
 * This thread fetches the data from the laser.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 */
SickTiM55xUSBAcquisitionThread::SickTiM55xUSBAcquisitionThread(std::string &cfg_name,
							       std::string &cfg_prefix)
  : SickTiM55xCommonAcquisitionThread(cfg_name, cfg_prefix)
{
  set_name("SickTiM55xUSB(%s)", cfg_name.c_str());
  usb_device_handle_ = NULL;
}

void
SickTiM55xUSBAcquisitionThread::init()
{
  read_common_config();

  try {
    cfg_serial_ = config->get_string((cfg_prefix_ + "serial").c_str());
  } catch (Exception &e) {} // ignore, if there is only one take that

  int usb_rv = 0;
  if ((usb_rv = libusb_init(&usb_ctx_)) != 0) {
    throw Exception("Failed to init libusb: %s", libusb_strerror((libusb_error)usb_rv));
  }
#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000106)
  libusb_set_option(usb_ctx_, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_ERROR);
#else
  libusb_set_debug(usb_ctx_, LIBUSB_LOG_LEVEL_ERROR);
#endif

  usb_mutex_ = new Mutex();

  try {
    init_device();
  } catch (...) {
    libusb_exit(usb_ctx_);
    throw;
  }

  pre_init(config, logger);
}


void
SickTiM55xUSBAcquisitionThread::finalize()
{
  if (usb_device_handle_) {
    try {
      const char *req_scan_data = "\x02sEN LMDscandata 0\x03";
      send_with_reply(req_scan_data);
    } catch (Exception &e) {} // ignore

    int usb_rv = 0;
    if ((usb_rv = libusb_release_interface(usb_device_handle_, 0)) != 0) {
      logger->log_warn(name(), "Sick TiM55x: failed to release device");
    }
    libusb_close(usb_device_handle_);
  }
  libusb_exit(usb_ctx_);

  free(_distances);
  _distances = NULL;

  free(_echoes);
  _echoes = NULL;

  delete usb_mutex_;
}


void
SickTiM55xUSBAcquisitionThread::loop()
{
  int actual_length = 0;
  size_t recv_buf_size = 32*1024;
  unsigned char recv_buf[recv_buf_size];

  if (usb_device_handle_) {
    MutexLocker lock(usb_mutex_);
    int usb_rv = 0;
    usb_rv = libusb_bulk_transfer(usb_device_handle_, (1 | LIBUSB_ENDPOINT_IN),
				  recv_buf, recv_buf_size - 1, &actual_length,
				  USB_TIMEOUT);
    if (usb_rv != 0) {
      if (usb_rv == LIBUSB_ERROR_NO_DEVICE) {
	logger->log_error(name(), "Device disconnected, will try to reconnect");
	libusb_close(usb_device_handle_);
	usb_device_handle_ = NULL;
      } else {
	logger->log_warn(name(), "Failed to read Sick TiM55x data (%d): %s",
			 usb_rv, libusb_strerror((libusb_error)usb_rv));
      }
      reset_distances();
      reset_echoes();
      return;
    } else {
      recv_buf[actual_length] = 0;
      lock.unlock();

      reset_distances();
      reset_echoes();

      try {
	parse_datagram(recv_buf, actual_length);
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to parse datagram, resyncing, exception follows");
	logger->log_warn(name(), e);
	resync();
      }
    }
  } else {
    try {
      init_device();
      logger->log_warn(name(), "Reconnected to device");
    } catch (Exception &e) {
      // ignore, keep trying
      usleep(USB_TIMEOUT * 1000);
      return;
    }
  }

  yield();
}


void
SickTiM55xUSBAcquisitionThread::open_device()
{
  if (usb_device_handle_)  return;

  libusb_device **devices;
  ssize_t num_devs = libusb_get_device_list(usb_ctx_, &devices);

  for (ssize_t i = 0; i < num_devs; ++i)
  {
    libusb_device_descriptor desc;
    int usb_rv = libusb_get_device_descriptor(devices[i], &desc);
    if (usb_rv != 0)  continue;

    if (desc.idVendor == USB_VENDOR && desc.idProduct == USB_PRODUCT) {
      // found a device

      if (usb_device_handle_ != NULL) {
	libusb_close(usb_device_handle_);
	usb_device_handle_ = NULL;
	libusb_free_device_list(devices, 1);
	throw Exception("Two devices found, specify serial of device to use.");
      }

      if ((usb_rv = libusb_open(devices[i], &usb_device_handle_)) != 0) {
	logger->log_warn(name(), "Failed to open Sick TiM55x: %s",
			 libusb_strerror((libusb_error) usb_rv));
	continue;
      }

      if (cfg_serial_ != "") {
	if (desc.iSerialNumber == 0) {
	  continue;
	}

	// read serial from device
	unsigned char serial_desc[32];
	usb_rv = libusb_get_string_descriptor_ascii(usb_device_handle_, desc.iSerialNumber,
						    serial_desc, 32);

	if (usb_rv <= 0) {
	  logger->log_warn(name(), "Failed to read serial from Sick TiM55x: %s",
			   libusb_strerror((libusb_error) usb_rv));
	  libusb_close(usb_device_handle_);
	  usb_device_handle_ = NULL;
	  continue;
	}

	std::string serial_desc_s((const char *)serial_desc, usb_rv);

	if (cfg_serial_ == serial_desc_s) {
	  break;
	} else {
	  logger->log_info(name(), "Ignoring Sick TiM55x with non-matching serial %s"
			   " (looking for %s)",
			   serial_desc_s.c_str(), cfg_serial_.c_str());
	  libusb_close(usb_device_handle_);
	  usb_device_handle_ = NULL;
	}
      }
    }
  }

  libusb_free_device_list(devices, 1);

  if (usb_device_handle_ != NULL) {
    int usb_rv;
    if (libusb_kernel_driver_active(usb_device_handle_, 0) == 1) {
      logger->log_info(name(), "Kernel driver active, disabling");
      if ((usb_rv = libusb_detach_kernel_driver(usb_device_handle_, 0)) != 0) {
	libusb_close(usb_device_handle_);
	usb_device_handle_ = NULL;
	throw Exception("Sick TiM55x: failed to detach kernel driver (%s)",
			libusb_strerror((libusb_error)usb_rv));
      }
    }

    if ((usb_rv = libusb_claim_interface(usb_device_handle_, 0)) != 0) {
      libusb_close(usb_device_handle_);
      usb_device_handle_ = NULL;
      throw Exception("Sick TiM55x: failed to claim device (%s)",
		      libusb_strerror((libusb_error)usb_rv));
    }
  } else {
    throw Exception("No matching device found");
  }
}


void
SickTiM55xUSBAcquisitionThread::close_device()
{
  libusb_release_interface(usb_device_handle_, 0);
  libusb_close(usb_device_handle_);
  usb_device_handle_ = NULL;
}

void
SickTiM55xUSBAcquisitionThread::flush_device()
{
  if (usb_device_handle_) {
    MutexLocker lock(usb_mutex_);
    int usb_rv = 0;
    int actual_length = 0;
    size_t recv_buf_size = 32*1024;
    unsigned char recv_buf[recv_buf_size];
    do {
      usb_rv = libusb_bulk_transfer(usb_device_handle_, (1 | LIBUSB_ENDPOINT_IN),
				    recv_buf, recv_buf_size - 1, &actual_length,
				    USB_TIMEOUT);

      // we don't care, we just want to get rid of data
    } while (usb_rv == 0 && actual_length > 0);
  }
}


void
SickTiM55xUSBAcquisitionThread::send_with_reply(const char *request,
						std::string *reply)
{
  MutexLocker lock(usb_mutex_);

  int usb_rv = 0;
  int actual_length = 0;
  int request_length = strlen(request);

  usb_rv = libusb_bulk_transfer(usb_device_handle_, (2 | LIBUSB_ENDPOINT_OUT),
				(unsigned char *)request, request_length,
				&actual_length, USB_TIMEOUT);
  if (usb_rv != 0 || actual_length != request_length) {
    throw Exception("Sick TiM55x: failed to send request (%s)",
		    libusb_strerror((libusb_error)usb_rv));
  }
 
  unsigned char tmpbuf[32*1024];
  usb_rv = libusb_bulk_transfer(usb_device_handle_, (1 | LIBUSB_ENDPOINT_IN),
				tmpbuf, 32*1024, &actual_length, USB_TIMEOUT);
  if (usb_rv != 0) {
    throw Exception("Sick TiM55x: failed to read reply (%s)",
		    libusb_strerror((libusb_error)usb_rv));
  }

  if (reply) {
    *reply = std::string((const char *)tmpbuf, actual_length);
  }
}
