
/***************************************************************************
 *  openprs_server_proxy.h - OpenPRS server proxy
 *
 *  Created: Tue Aug 19 16:59:27 2014
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

#ifndef __PLUGINS_OPENPRS_UTILS_OPENPRS_SERVER_PROXY_H_
#define __PLUGINS_OPENPRS_UTILS_OPENPRS_SERVER_PROXY_H_

#include <core/utils/lockptr.h>
#include <core/utils/lock_list.h>

#include <boost/asio.hpp>
#include <list>
#include <string>
#include <thread>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

class OpenPRSServerProxy
{
 public:
  OpenPRSServerProxy(unsigned short tcp_port,
		     const std::string &server_host, unsigned short server_port,
		     fawkes::Logger *logger);
  virtual ~OpenPRSServerProxy();

  void transmit_command(const std::string &client_name, const std::string &command);
  void transmit_command_f(const std::string &client_name, const char *format, ...);
  void transmit_command_v(const std::string &client_name, const char *format, va_list arg);

  bool has_kernel(const std::string &kernel_name);

  static int         read_int_from_socket(boost::asio::ip::tcp::socket &socket);
  static std::string read_string_from_socket(boost::asio::ip::tcp::socket &socket);
  static void        write_int_to_socket(boost::asio::ip::tcp::socket &socket, int i);
  static void        write_string_to_socket(boost::asio::ip::tcp::socket &socket, const std::string &str);
  static void        write_string_newline_to_socket(boost::asio::ip::tcp::socket &socket, const std::string &str);

 private:
  class Mapping {
  public:
    /** Shortcut for shared pointer of session. */
    typedef std::shared_ptr<Mapping> Ptr;
    Mapping(boost::asio::io_service &io_service,
	    const std::string &server_host, unsigned short server_port,
	    fawkes::Logger *logger);
    ~Mapping();

    void start();
    bool alive() const;
    void disconnect();

    void transmit_command(const std::string &command);

  private: // methods
    void handle_resolve(const boost::system::error_code& err,
			boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
    void handle_connect(const boost::system::error_code& err);
    void start_recv_client();
    void start_recv_server();
    void handle_recv_client(const boost::system::error_code &err);
    void handle_recv_server(const boost::system::error_code &err);

  private: // members
    boost::asio::io_service        &io_service_;
    boost::asio::ip::tcp::resolver  resolver_;

    std::string                     server_host_;
    unsigned short                  server_port_;

    int                             client_in_num_completions_;
    boost::asio::streambuf          server_buffer_;

    fawkes::Logger                 *logger_;

  public:
    std::string                   client_name;
    boost::asio::ip::tcp::socket  client_socket;
    boost::asio::ip::tcp::socket  server_socket;

  };

 private:
  void start_accept();
  void handle_accept(Mapping::Ptr mapping, const boost::system::error_code &error);
  Mapping::Ptr find_mapping(const std::string &client_name);

 private:
  boost::asio::io_service               io_service_;
  std::thread                           io_service_thread_;
  boost::asio::io_service::work         io_service_work_;
  boost::asio::ip::tcp::acceptor        acceptor_;

  std::string               server_host_;
  unsigned short            server_port_;
  Logger                   *logger_;

  fawkes::LockList<Mapping::Ptr>   mappings_;
};

} // end namespace fawkes

#endif
