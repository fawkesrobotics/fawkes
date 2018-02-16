
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

#include "sick_tim55x_common_aqt.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/misc/string_split.h>
#include <utils/math/angle.h>

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>

using namespace fawkes;


/** @class SickTiM55xCommonAcquisitionThread "sick_tim55x_common_aqt.h"
 * Laser acqusition thread for Sick TiM55x laser range finders.
 * This thread fetches the data from the laser.
 * @author Tim Niemueller
 *
 * @fn void SickTiM55xCommonAcquisitionThread::send_with_reply(const char *request, std::string *reply = NULL)
 * Send a request and expect a reply.
 * @param request request to send
 * @param reply upon returns contains the received reply, maybe NULL to ignore the reply
 *
 * @fn void SickTiM55xCommonAcquisitionThread::open_device()
 * Open the device.
 * Virtual method implemented by the actual connection driver.
 *
 * @fn void SickTiM55xCommonAcquisitionThread::close_device()
 * Close the device.
 * Virtual method implemented by the actual connection driver.
 *
 * @fn void SickTiM55xCommonAcquisitionThread::flush_device()
 * Flush the device.
 * Read all current data on the channel and return on no data to read
 * or timeout.
 *
 * @var std::string SickTiM55xCommonAcquisitionThread::cfg_name_
 * Name of the particular configuration instance.
 *
 * @var std::string SickTiM55xCommonAcquisitionThread::cfg_prefix_
 * Configuration path prefix for this configuration.
 *
 * @var std::string SickTiM55xCommonAcquisitionThread::dev_model_
 * Device model type as string.
 */


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 */
SickTiM55xCommonAcquisitionThread::SickTiM55xCommonAcquisitionThread(std::string &cfg_name,
								     std::string &cfg_prefix)
  : LaserAcquisitionThread("SickTiM55xCommonAcquisitionThread"),
    ConfigurationChangeHandler(cfg_prefix.c_str())
{
  set_name("SickTiM55x(%s)", cfg_name.c_str());
  pre_init_done_ = false;
  cfg_name_      = cfg_name;
  cfg_prefix_    = cfg_prefix;
}


/** Destructor. */
SickTiM55xCommonAcquisitionThread::~SickTiM55xCommonAcquisitionThread()
{
}

void
SickTiM55xCommonAcquisitionThread::pre_init(fawkes::Configuration *config,
					 fawkes::Logger        *logger)
{
  if (pre_init_done_)  return;
  pre_init_done_ = true;

  if (dev_model_.empty()) {
    throw Exception("LaserSick5xx: model has not yet been determined");
  }

  if (dev_model_ == "TiM5xx") {
    _distances_size = 360;
    _echoes_size = 360;
    expected_num_data_ = 271;
  } else if (dev_model_ == "TiM571") {
    _distances_size = 1080;
    _echoes_size = 1080;
    expected_num_data_ = 811;
  } else {
    throw Exception("LaserSick5xx: unknown model %s", dev_model_.c_str());
  }

  alloc_distances(_distances_size);
  alloc_echoes(_echoes_size);

  config->add_change_handler(this);
}

/** Read common configuration parameters. */
void
SickTiM55xCommonAcquisitionThread::read_common_config()
{
  cfg_time_offset_ = 0.;
  try {
    cfg_time_offset_ += config->get_float((cfg_prefix_ + "time_offset").c_str());
  } catch (Exception &e) {} // ignored, use default
  logger->log_debug(name(), "Time offset: %f", cfg_time_offset_);
}


/** Initialize device. */
void
SickTiM55xCommonAcquisitionThread::init_device()
{
  open_device();

  // turn off data transfer, just in case...
  try {
    const char *req_scan_data = "\x02sEN LMDscandata 0\x03";
    send_with_reply(req_scan_data);
  } catch (Exception &e) {} // ignore

  flush_device();

  std::string rep_dev_indent;
  try {
    const char *req_dev_indent = "\x02sRI0\x03\0";
    send_with_reply(req_dev_indent, &rep_dev_indent);
  } catch (Exception &e) {
    close_device();
    e.append("Failed to get device indent");
    throw;
  }
  rep_dev_indent += '\0';
  rep_dev_indent = rep_dev_indent.substr(9, rep_dev_indent.length() - 11);
  dev_model_ = rep_dev_indent.substr(0, rep_dev_indent.find(" "));
  logger->log_debug(name(), "Ident: %s", rep_dev_indent.c_str());

  try {
    const char *req_scan_data = "\x02sEN LMDscandata 1\x03";
    send_with_reply(req_scan_data);
  } catch (Exception &e) {
    close_device();
    e.append("Failed to start data streaming");
    throw;
  }  
}


/** Resynchronize to laser data.
 * Stop data transfer, flush, restart.
 */
void
SickTiM55xCommonAcquisitionThread::resync()
{
  // turn off data transfer
  try {
    const char *req_scan_data = "\x02sEN LMDscandata 0\x03";
    send_with_reply(req_scan_data);
  } catch (Exception &e) {} // ignore

  flush_device();

  // turn on data transfer
  try {
    const char *req_scan_data = "\x02sEN LMDscandata 1\x03";
    send_with_reply(req_scan_data);
  } catch (Exception &e) {} // ignore

}

/** Parse incoming message from device.
 * Based on https://www.mysick.com/saqqara/pdf.aspx?id=im0053129 and
 * https://github.com/uos/sick_tim3xx.
 * @param datagram data content
 * @param datagram_length length in bytes of @p datagram
 */
void
SickTiM55xCommonAcquisitionThread::parse_datagram(const unsigned char *datagram,
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

  if (number_of_data != expected_num_data_) {
    throw Exception("Invalid data length, got %u, expected %u",
		    number_of_data, expected_num_data_);
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
  unsigned short scanning_freq = -1;
  sscanf(fields[16].c_str(), "%hx", &scanning_freq);
  float scan_time = 1.0 / (scanning_freq / 100.0);

  // 17: Measurement Frequency (36)
  // this yields wrong results on some devices
  //unsigned short measurement_freq = -1;
  //sscanf(fields[17].c_str(), "%hx", &measurement_freq);
  //float time_increment = 1.0 / (measurement_freq * 100.0);

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
  float angle_increment_deg = rad2deg(angle_increment);
  //float angle_max = angle_min + (number_of_data - 1) * angle_increment;

  // 25: Number of data (<= 10F)
  // This is already determined above in number_of_data

  // 26..26 + n - 1: Data_1 .. Data_n
  _data_mutex->lock();
  _timestamp->stamp();

  int start_idx = (int)roundf(rad2deg(angle_min) / angle_increment_deg);

  for (int j = 0; j < number_of_data; ++j) {
    unsigned short range;
    sscanf(fields[j + 26].c_str(), "%hx", &range);
    int idx = (_distances_size + start_idx + j) % _distances_size;
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
      int idx = (_echoes_size + start_idx + j) % _echoes_size;
      _echoes[idx] = intensity;
    }
  }

  _new_data = true;

  float time_increment = scan_time * angle_increment / (2.0 * M_PI);

  *_timestamp -= number_of_data * time_increment;
  *_timestamp += cfg_time_offset_;

  _data_mutex->unlock();

  // 26 + n: RSSI data included
  // IF RSSI not included:
  //   26 + n + 1 .. 26 + n + 3 = unknown (but seems to be [0, 1, B] always)
  //   26 + n + 4 .. count - 4 = device label
  //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
  //   <ETX> (\x03)
}

void
SickTiM55xCommonAcquisitionThread::config_value_changed(
  const fawkes::Configuration::ValueIterator *v)
{
  MutexLocker lock(loop_mutex);
  read_common_config();
}

void
SickTiM55xCommonAcquisitionThread::config_value_erased(const char *path)
{
  MutexLocker lock(loop_mutex);
  read_common_config();
}
