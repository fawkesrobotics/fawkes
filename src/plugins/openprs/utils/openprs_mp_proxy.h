
/***************************************************************************
 *  openprs_mp_proxy.h - OpenPRS message passer proxy
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

#ifndef __PLUGINS_OPENPRS_UTILS_OPENPRS_MP_PROXY_H_
#define __PLUGINS_OPENPRS_UTILS_OPENPRS_MP_PROXY_H_

#include <core/utils/lockptr.h>

#include <boost/asio.hpp>
#include <list>
#include <string>
#include <thread>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

class OpenPRSMessagePasserProxy
{
 public:
  OpenPRSMessagePasserProxy(unsigned short tcp_port,
			    const std::string &mp_host, unsigned short mp_port,
			    fawkes::Logger *logger);
  virtual ~OpenPRSMessagePasserProxy();

 private:
  class Mapping {
  public:
    /** Shortcut for shared pointer of session. */
    typedef std::shared_ptr<Mapping> Ptr;
    Mapping(boost::asio::io_service &io_service,
	    const std::string &mp_host, unsigned short mp_port,
	    fawkes::Logger *logger);
    ~Mapping();

    void start();
    bool alive() const;
    void disconnect();

  private: // methods
    void disconnect(const char *where, const char *reason);
    void handle_resolve(const boost::system::error_code& err,
			       boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
    void handle_connect(const boost::system::error_code& err);
    void start_recv_client();
    void handle_recv_client(const boost::system::error_code &err);
    void start_recv_server();
    void handle_recv_server_reg_reply(const boost::system::error_code &err);
    void handle_recv_server_message_pt(const boost::system::error_code &err);
    void handle_recv_server_strings_pt(const boost::system::error_code &err);

    int         read_int_from_socket(boost::asio::ip::tcp::socket &socket);
    std::string read_string_from_socket(boost::asio::ip::tcp::socket &socket);
    void        write_int_to_socket(boost::asio::ip::tcp::socket &socket, int i);
    void        write_string_to_socket(boost::asio::ip::tcp::socket &socket, std::string &str);
    void        write_string_newline_to_socket(boost::asio::ip::tcp::socket &socket,
					       const std::string &str);
  private: // members
    boost::asio::io_service        &io_service_;
    boost::asio::ip::tcp::resolver  resolver_;

    std::string                     server_host_;
    unsigned short                  server_port_;
    boost::asio::streambuf          server_buffer_;

    fawkes::Logger                 *logger_;

    int                             server_in_reg_reply_;
    int                             server_in_str_len_;
    int                             client_in_msg_type_;

  public:
    int                           client_prot;
    std::string                   client_name;
    boost::asio::ip::tcp::socket  client_socket;
    boost::asio::ip::tcp::socket  server_socket;

  };

 private:
  void start_accept();
  void handle_accept(Mapping::Ptr mapping, const boost::system::error_code &error);

 private:
  boost::asio::io_service               io_service_;
  std::thread                           io_service_thread_;
  boost::asio::io_service::work         io_service_work_;
  boost::asio::ip::tcp::acceptor        acceptor_;

  std::string               mp_host_;
  unsigned short            mp_port_;
  Logger                   *logger_;

  std::list<Mapping::Ptr>   mappings_;
};

} // end namespace fawkes

#endif
