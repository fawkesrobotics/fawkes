
/***************************************************************************
 *  sick_tim55x_ethernet_aqt.h - Retrieve data from Sick TiM 55x via Ethernet
 *
 *  Created: Sun Jun 15 20:44:32 2014
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

#ifndef __PLUGINS_LASER_SICK_TIM55X_ETHERNET_AQT_H_
#define __PLUGINS_LASER_SICK_TIM55X_ETHERNET_AQT_H_

#include "sick_tim55x_common_aqt.h"

#include <string>
#include <boost/asio.hpp>

namespace fawkes {
  class Mutex;
}

class SickTiM55xEthernetAcquisitionThread : public SickTiM55xCommonAcquisitionThread
{
 public:
  SickTiM55xEthernetAcquisitionThread(std::string &cfg_name, std::string &cfg_prefix);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  void open_device();
  void close_device();
  void flush_device();
  void send_with_reply(const char *request, std::string *reply = NULL);

  void check_deadline();
  void check_soft_timeout();

#if BOOST_VERSION < 104800
  void handle_read(boost::system::error_code ec, size_t bytes_read)
  {
    ec_ = ec;
    bytes_read_ = bytes_read;
  }
#endif

 private:
  std::string  cfg_host_;
  std::string  cfg_port_;

  fawkes::Mutex  *socket_mutex_;

  boost::asio::io_service       io_service_;
  boost::asio::ip::tcp::socket  socket_;
  boost::asio::deadline_timer   deadline_;
  boost::asio::deadline_timer   soft_deadline_;
  boost::asio::streambuf        input_buffer_;

  boost::system::error_code ec_;
  size_t bytes_read_;
};


#endif
