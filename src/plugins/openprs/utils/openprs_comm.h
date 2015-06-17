
/***************************************************************************
 *  openprs_comm.h - OpenPRS communication wrapper for Fawkes
 *
 *  Created: Mon Aug 18 14:33:55 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __PLUGINS_OPENPRS_ASPECT_OPENPRS_COMM_H_
#define __PLUGINS_OPENPRS_ASPECT_OPENPRS_COMM_H_

#include <vector>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenPRSServerProxy;
class Logger;

class OpenPRSComm
{
 public:
  OpenPRSComm(const char *local_name, const char *hostname, unsigned short port,
	      OpenPRSServerProxy *server_proxy, Logger *logger = NULL);
  virtual ~OpenPRSComm();

  /** Get OpenPRS local name.
   * @return OpenPRS local name */
  const std::string & name() const
  { return name_; }

  void send_message(const char *recipient, const char *message);
  void broadcast_message(const char *message);
  void multicast_message(std::vector<const char *> &recipients, const char *message);

  void send_message(const std::string &recipient, const std::string &message);
  void broadcast_message(const std::string &message);
  void multicast_message(const std::vector<std::string> &recipients, const std::string &message);

  void send_message_f(const std::string &recipient, const char *format, ...);
  void broadcast_message_f(const char *format, ...);
  void multicast_message_f(const std::vector<std::string> &recipients, const char *format, ...);

  void transmit_command(const char *recipient, const char *message);
  void transmit_command(const std::string &recipient, const std::string &message);
  void transmit_command_f(const std::string &recipient, const char *format, ...);

  /** Boost signal for a received message. */
  typedef
    boost::signals2::signal<void (std::string, std::string)>
    signal_msg_rcvd_type;

  /** Signal that is invoked when a message has been received.
   * @return signal
   */
  signal_msg_rcvd_type &  signal_msg_rcvd()
  { return sig_rcvd_; }

 private: // methods
  void start_recv();
  void handle_recv(const boost::system::error_code &err);
  std::string read_string_from_socket(boost::asio::posix::stream_descriptor &socket);


 private: // members
  const std::string   name_;
  int                 mp_socket_;
  OpenPRSServerProxy *server_proxy_;
  Logger             *logger_;

  boost::asio::io_service               io_service_;
  std::thread                           io_service_thread_;
  boost::asio::io_service::work         io_service_work_;
  boost::asio::posix::stream_descriptor sd_mp_socket_;

  signal_msg_rcvd_type                  sig_rcvd_;
};

} // end namespace fawkes

#endif
