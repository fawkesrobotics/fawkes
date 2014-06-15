
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
  : LaserAcquisitionThread("SickTiM55xUSBAcquisitionThread")
{
  set_name("SickTiM55x(%s)", cfg_name.c_str());
  __pre_init_done = false;
  __cfg_name   = cfg_name;
  __cfg_prefix = cfg_prefix;
}


void
SickTiM55xUSBAcquisitionThread::pre_init(fawkes::Configuration *config,
				      fawkes::Logger        *logger)
{
  if (__pre_init_done)  return;

  _distances_size = 360;

  __pre_init_done = true;
}

void
SickTiM55xUSBAcquisitionThread::init()
{
  pre_init(config, logger);

  int usb_rv = 0;
  if ((usb_rv = libusb_init(&usb_ctx_)) != 0) {
    throw Exception("Failed to init libusb: %s", libusb_strerror((libusb_error)usb_rv));
  }

  libusb_set_debug(usb_ctx_, LIBUSB_LOG_LEVEL_ERROR);

  try {
    __cfg_serial = config->get_string((__cfg_prefix + "serial").c_str());
  } catch (Exception &e) {} // ignore, if there is only one take that

  __cfg_time_offset = 0.;
  try {
    __cfg_time_offset += config->get_float((__cfg_prefix + "time_offset").c_str());
  } catch (Exception &e) {} // ignored, use default

  usb_mutex_ = new Mutex();
  try {
    init_device();
  } catch (...) {
    delete usb_mutex_;
    libusb_exit(usb_ctx_);
    throw;
  }
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

      parse_datagram(recv_buf, actual_length);
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


libusb_device_handle *
SickTiM55xUSBAcquisitionThread::open_device(uint16_t vendor_id, uint16_t product_id,
					 std::string &serial)
{
  libusb_device **devices;
  ssize_t num_devs = libusb_get_device_list(usb_ctx_, &devices);

  libusb_device_handle *device_handle = NULL;

  for (ssize_t i = 0; i < num_devs; ++i)
  {
    libusb_device_descriptor desc;
    int usb_rv = libusb_get_device_descriptor(devices[i], &desc);
    if (usb_rv != 0)  continue;

    if (desc.idVendor == vendor_id && desc.idProduct == product_id) {
      // found a device

      if (device_handle != NULL) {
	libusb_close(device_handle);
	libusb_free_device_list(devices, 1);
	throw Exception("Two devices found, specify serial of device to use.");
      }

      if ((usb_rv = libusb_open(devices[i], &device_handle)) != 0) {
	logger->log_warn(name(), "Failed to open Sick TiM55x: %s",
			 libusb_strerror((libusb_error) usb_rv));
	continue;
      }

      if (serial != "") {
	if (desc.iSerialNumber == 0) {
	  continue;
	}

	// read serial from device
	unsigned char serial_desc[32];
	usb_rv = libusb_get_string_descriptor_ascii(device_handle, desc.iSerialNumber,
						    serial_desc, 32);

	if (usb_rv <= 0) {
	  logger->log_warn(name(), "Failed to read serial from Sick TiM55x: %s",
			   libusb_strerror((libusb_error) usb_rv));
	  libusb_close(device_handle);
	  device_handle = NULL;
	  continue;
	}

	std::string serial_desc_s((const char *)serial_desc, usb_rv);

	if (serial == serial_desc_s) {
	  libusb_free_device_list(devices, 1);
	  return device_handle;
	} else {
	  logger->log_info(name(), "Ignoring Sick TiM55x with non-matching serial %s"
			   " (looking for %s)",
			   serial_desc_s.c_str(), serial.c_str());
	  libusb_close(device_handle);
	  device_handle = NULL;
	}
      }
    }
  }

  libusb_free_device_list(devices, 1);

  if (device_handle != NULL) {
    return device_handle;
  } else {
    throw Exception("No matching device found");
  }
}

void
SickTiM55xUSBAcquisitionThread::init_device()
{
  if (usb_device_handle_)  return;

  usb_device_handle_ = open_device(USB_VENDOR, USB_PRODUCT, __cfg_serial);

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

  // turn off data transfer, just in case...
  try {
    const char *req_scan_data = "\x02sEN LMDscandata 0\x03";
    send_with_reply(req_scan_data);
  } catch (Exception &e) {} // ignore

  std::string rep_dev_indent;
  try {
    const char *req_dev_indent = "\x02sRI0\x03\0";
    send_with_reply(req_dev_indent, &rep_dev_indent);
  } catch (Exception &e) {
    libusb_release_interface(usb_device_handle_, 0);
    libusb_close(usb_device_handle_);
    usb_device_handle_ = NULL;
    e.append("Failed to get device indent");
    throw;
  }
  rep_dev_indent += '\0';
  rep_dev_indent = rep_dev_indent.substr(9, rep_dev_indent.length() - 11);
  logger->log_debug(name(), "Ident: %s", rep_dev_indent.c_str());

  alloc_distances(_distances_size);

  try {
    const char *req_scan_data = "\x02sEN LMDscandata 1\x03";
    send_with_reply(req_scan_data);
  } catch (Exception &e) {
    libusb_release_interface(usb_device_handle_, 0);
    libusb_close(usb_device_handle_);
    usb_device_handle_ = NULL;
    e.append("Failed to start data streaming");
    throw;
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

/** Parse incoming message from device.
 * Based on https://www.mysick.com/saqqara/pdf.aspx?id=im0053129 and
 * https://github.com/uos/sick_tim3xx.
 * @param datagram data content
 * @param datagram_length length in bytes of @p datagram
 */
void
SickTiM55xUSBAcquisitionThread::parse_datagram(const unsigned char *datagram,
					       size_t datagram_length)
{
  static const size_t HEADER_FIELDS = 33;

  std::string datagram_s((const char *)datagram, datagram_length);
  std::vector<std::string> fields = str_split(datagram_s, ' ');

  size_t count = fields.size();

  // Validate header. Total number of tokens is highly unreliable as this may
  // change when you change the scanning range or the device name using SOPAS ET
  // tool. The header remains stable, however.
  if (count < HEADER_FIELDS) {
    throw Exception("Insufficient number of fields received");
  }
  if (fields[15] != "0") {
    throw Exception("Invalid datagram format, ignoring scan");
  }
  if (fields[20] != "DIST1") {
    throw Exception("Invalid datagram format (DIST1), ignoring scan");
  }

  // More in depth checks: check data length and RSSI availability
  // 25: Number of data (<= 10F)
  unsigned short int number_of_data = 0;
  sscanf(fields[25].c_str(), "%hx", &number_of_data);

  if (number_of_data < 1 || number_of_data > 271) {
    throw Exception("Invalid data length %u not in [1..271]", number_of_data);
  }
  if (count < HEADER_FIELDS + number_of_data) {
    throw Exception("Invalid number of fields received, got %zu, expected %u+%u=%u",
		    count, HEADER_FIELDS, number_of_data, HEADER_FIELDS + number_of_data);
  }

  // Calculate offset of field that contains indicator of whether or not RSSI data is included
  size_t rssi_idx = 26 + number_of_data;
  int tmp;
  sscanf(fields[rssi_idx].c_str(), "%d", &tmp);
  bool rssi = tmp > 0;
  unsigned short int number_of_rssi_data = 0;
  if (rssi) {
    sscanf(fields[rssi_idx + 6].c_str(), "%hx", &number_of_rssi_data);

    // Number of RSSI data should be equal to number of data
    if (number_of_rssi_data != number_of_data) {
      throw Exception("Number of RSSI data is lower than number of range data (%d vs %d)",
		      number_of_data, number_of_rssi_data);
    }

    // Check if the total length is still appropriate.
    // RSSI data size = number of RSSI readings + 6 fields describing the data
    if (count < HEADER_FIELDS + number_of_data + number_of_rssi_data + 6) {
      throw Exception("Less fields than expected for %d data points (%zu)",
		      number_of_data, count);
    }

    if (fields[rssi_idx + 1] != "RSSI1") {
      throw Exception("Field %zu of received data is not equal to RSSI1 (%s)",
		      rssi_idx + 1, fields[rssi_idx + 1].c_str());
    }
  }

  // <STX> (\x02)
  // 0: Type of command (SN)
  // 1: Command (LMDscandata)
  // 2: Firmware version number (1)
  // 3: Device number (1)
  // 4: Serial number (eg. B96518)
  // 5 + 6: Device Status (0 0 = ok, 0 1 = error)
  // 7: Telegram counter (eg. 99)
  // 8: Scan counter (eg. 9A)
  // 9: Time since startup (eg. 13C8E59)
  // 10: Time of transmission (eg. 13C9CBE)
  // 11 + 12: Input status (0 0)
  // 13 + 14: Output status (8 0)
  // 15: Reserved Byte A (0)

  // 16: Scanning Frequency (5DC)
  //unsigned short scanning_freq = -1;
  //sscanf(fields[16], "%hx", &scanning_freq);
  //scan_time = 1.0 / (scanning_freq / 100.0);

  // 17: Measurement Frequency (36)
  unsigned short measurement_freq = -1;
  sscanf(fields[17].c_str(), "%hx", &measurement_freq);
  float time_increment = 1.0 / (measurement_freq * 100.0);

  // 18: Number of encoders (0)
  // 19: Number of 16 bit channels (1)
  // 20: Measured data contents (DIST1)

  // 21: Scaling factor (3F800000)
  // ignored for now (is always 1.0):
  // unsigned int scaling_factor_int = -1;
  // sscanf(fields[21], "%x", &scaling_factor_int);
  // float scaling_factor = reinterpret_cast<float&>(scaling_factor_int);

  // 22: Scaling offset (00000000) -- always 0
  // 23: Starting angle (FFF92230)
  int starting_angle_val = -1;
  sscanf(fields[23].c_str(), "%x", &starting_angle_val);
  float angle_min = (starting_angle_val / 10000.0) / 180.0 * M_PI - M_PI / 2;

  // 24: Angular step width (2710)
  unsigned short angular_step_width = -1;
  sscanf(fields[24].c_str(), "%hx", &angular_step_width);
  float angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;
  //float angle_max = angle_min + (number_of_data - 1) * angle_increment;

  // 25: Number of data (<= 10F)
  // This is already determined above in number_of_data

  // 26..26 + n - 1: Data_1 .. Data_n
  _data_mutex->lock();
  _timestamp->stamp();
  for (int j = 0; j < number_of_data; ++j) {
    unsigned short range;
    sscanf(fields[j + 26].c_str(), "%hx", &range);
    int idx = (360 + ((int)roundf(rad2deg(angle_min + j * angle_increment)))) % 360;
    _distances[idx] = range / 1000.0;
  }

  if (rssi) {
    // 26 + n: RSSI data included

    //   26 + n + 1 = RSSI Measured Data Contents (RSSI1)
    //   26 + n + 2 = RSSI scaling factor (3F80000)
    //   26 + n + 3 = RSSI Scaling offset (0000000)
    //   26 + n + 4 = RSSI starting angle (equal to Range starting angle)
    //   26 + n + 5 = RSSI angular step width (equal to Range angular step width)
    //   26 + n + 6 = RSSI number of data (equal to Range number of data)
    //   26 + n + 7 .. 26 + n + 7 + n - 1: RSSI_Data_1 .. RSSI_Data_n
    //   26 + n + 7 + n .. 26 + n + 7 + n + 2 = unknown (but seems to be [0, 1, B] always)
    //   26 + n + 7 + n + 2 .. count - 4 = device label
    //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
    //   <ETX> (\x03)
    size_t offset = 26 + number_of_data + 7;
    for (int j = 0; j < number_of_data; ++j) {
      unsigned short intensity;
      sscanf(fields[j + offset].c_str(), "%hx", &intensity);
      int idx = (360 + ((int)roundf(rad2deg(angle_min + j * angle_increment)))) % 360;
      _echoes[idx] = intensity;
    }
  }

  _new_data = true;

  *_timestamp -= number_of_data * time_increment;
  *_timestamp += __cfg_time_offset;

  _data_mutex->unlock();

  // 26 + n: RSSI data included
  // IF RSSI not included:
  //   26 + n + 1 .. 26 + n + 3 = unknown (but seems to be [0, 1, B] always)
  //   26 + n + 4 .. count - 4 = device label
  //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
  //   <ETX> (\x03)
}
