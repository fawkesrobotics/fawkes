
/***************************************************************************
 *  imu_cruizcore_xg1010.h - Retrieve IMU data from CruizCore XG1010
 *
 *  Created: Sun Jun 22 21:41:38 2014
 *  Copyright  2008-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_IMU_IMU_CRUIZCORE_XG1010_H_
#define __PLUGINS_IMU_IMU_CRUIZCORE_XG1010_H_

#include "acquisition_thread.h"
#include "imu_cruizcore_xg1010.h"

#include <boost/asio.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <string>
#include <thread>

#define CRUIZCORE_XG1010_PACKET_SIZE 8

namespace fawkes {
  class Mutex;
  class Time;
#ifdef USE_TIMETRACKER
  class TimeTracker;
#endif
}

class CruizCoreXG1010AcquisitionThread : public IMUAcquisitionThread
{
 public:
  CruizCoreXG1010AcquisitionThread(std::string &cfg_name, std::string &cfg_prefix,
				   bool continuous);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  void open_device();
  void close_device();
  void resync();
  void send_init_packet(bool enable_transfer);

  void parse_packet();
  void check_deadline();

#if BOOST_VERSION < 104800
  void handle_read(boost::system::error_code ec, size_t bytes_read)
  {
    ec_ = ec;
    bytes_read_ = bytes_read;
  }
#endif

 private:
  std::string  cfg_serial_;
  unsigned int cfg_baud_rate_;
  unsigned int cfg_freq_;

  boost::asio::io_service       io_service_;
  boost::asio::serial_port      serial_;
  boost::asio::io_service::work io_service_work_;
  boost::asio::deadline_timer   deadline_;
  boost::asio::streambuf        input_buffer_;

  unsigned int  receive_timeout_;
  unsigned char in_packet_[CRUIZCORE_XG1010_PACKET_SIZE];

  boost::system::error_code ec_;
  size_t bytes_read_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker  *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_full_loop_;
  unsigned int ttc_read_;
  unsigned int ttc_catch_up_;
  unsigned int ttc_parse_;
#endif
};


#endif
