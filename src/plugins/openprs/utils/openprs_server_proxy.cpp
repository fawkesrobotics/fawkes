
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

#include "openprs_server_proxy.h"

#include <core/exception.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost::asio;

// Types copied from OPRS because they are not public there
/// @cond EXTERN
namespace OPRS {
  typedef enum {MESSAGE_MT = 1, BROADCAST_MT, MULTICAST_MT, DISCONNECT_MT } Message_Type;
  typedef enum {REGISTER_OK, REGISTER_NAME_CONFLICT, REGISTER_DENIED} Register_Type;
  typedef enum {MESSAGES_PT, STRINGS_PT} Protocol_Type;
}
/// @endcond

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenPRSServerProxy "openprs_server_proxy.h"
 * Proxy for the OpenPRS server communication.
 * Using this proxy allows to inject commands into the communication between
 * oprs-server and oprs (or xoprs).
 * @author Tim Niemueller
 */


/** Constructor.
 * @param tcp_port port to listen on for incoming connections
 * @param server_host host of oprs-server to connect to
 * @param server_port TCP port that oprs-server listens on
 * @param logger logger for informational messages
 */
OpenPRSServerProxy::OpenPRSServerProxy(unsigned short tcp_port,
				       const std::string &server_host, unsigned short server_port,
				       fawkes::Logger *logger)
  : io_service_work_(io_service_), acceptor_(io_service_, ip::tcp::endpoint(ip::tcp::v6(), tcp_port)),
    server_host_(server_host), server_port_(server_port), logger_(logger)
{
  acceptor_.set_option(socket_base::reuse_address(true));
  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });
  start_accept();
}


/** Destructor. */
OpenPRSServerProxy::~OpenPRSServerProxy()
{
  io_service_.stop();
  io_service_thread_.join();
}


/** Check if a kernel connected to the proxy.
 * @param kernel_name name of the kernel to look for
 * @return true if the kernel connected, false otherwise
 */
bool
OpenPRSServerProxy::has_kernel(const std::string &kernel_name)
{
  auto map_it = find_if(mappings_.begin(), mappings_.end(),
			[&kernel_name] (const Mapping::Ptr &mapping)
			  { return mapping->client_name == kernel_name; });
  return (map_it != mappings_.end());
}


OpenPRSServerProxy::Mapping::Ptr
OpenPRSServerProxy::find_mapping(const std::string &recipient)
{
  auto map_it = find_if(mappings_.begin(), mappings_.end(),
			[&recipient] (const Mapping::Ptr &mapping)
			  { return mapping->client_name == recipient; });
  if (map_it != mappings_.end()) {
    return *map_it;
  } else {
    throw Exception("Client %s is not connected to OpenPRS server proxy", recipient.c_str());
  }
}

/** Transmit a command to an OpenPRS kernel.
 * This works equivalent to the transmit oprs-server console command.
 * @param recipient OpenPRS kernel name to send to
 * @param command command to send, cf. OpenPRS manual for valid commands
 */
void
OpenPRSServerProxy::transmit_command(const std::string &recipient, const std::string &command)
{
  MutexLocker lock(mappings_.mutex());
  Mapping::Ptr mapping = find_mapping(recipient);
  mapping->transmit_command(command);
}


/** Transmit a command to an OpenPRS kernel.
 * This works equivalent to the transmit oprs-server console command.
 * This function allows to pass a format according to the sprintf()
 * format and its arguments.
 * @param recipient OpenPRS kernel name to send to
 * @param format format string for the command, must be followed by the
 * appropriate number and types of arguments.
 */
void
OpenPRSServerProxy::transmit_command_f(const std::string &recipient, const char *format, ...)
{
  MutexLocker lock(mappings_.mutex());
  Mapping::Ptr mapping = find_mapping(recipient);

  va_list arg;
  va_start(arg, format);

  char *msg;
  if (vasprintf(&msg, format, arg) == -1) {
    throw OutOfMemoryException("Cannot format OpenPRS client command string");
  }
  va_end(arg);
  std::string command = msg;
  free(msg);

  mapping->transmit_command(command);
}

/** Transmit a command to an OpenPRS kernel.
 * This works equivalent to the transmit oprs-server console command.
 * This function allows to pass a format according to the sprintf()
 * format and its arguments. The arguments are read from the @p arg list.
 * @param recipient OpenPRS kernel name to send to
 * @param format format string for the command, must be followed by the
 * appropriate number and types of arguments.
 * @param arg argument list for the string format
 */
void
OpenPRSServerProxy::transmit_command_v(const std::string &recipient, const char *format, va_list arg)
{
  MutexLocker lock(mappings_.mutex());
  Mapping::Ptr mapping = find_mapping(recipient);

  char *msg;
  if (vasprintf(&msg, format, arg) == -1) {
    throw OutOfMemoryException("Cannot format OpenPRS client command string");
  }
  std::string command = msg;
  free(msg);

  mapping->transmit_command(command);
}


/** Start accepting connections. */
void
OpenPRSServerProxy::start_accept()
{
  Mapping::Ptr mapping(new Mapping(io_service_, server_host_, server_port_, logger_));
  acceptor_.async_accept(mapping->client_socket,
			 boost::bind(&OpenPRSServerProxy::handle_accept, this,
				     mapping, boost::asio::placeholders::error));
}

void
OpenPRSServerProxy::handle_accept(Mapping::Ptr mapping,
				  const boost::system::error_code& error)
{
  if (! error) {
    MutexLocker lock(mappings_.mutex());
    mappings_.push_back(mapping);
    mapping->start();
  }

  start_accept();
}


OpenPRSServerProxy::Mapping::Mapping(boost::asio::io_service &io_service,
				     const std::string &server_host, unsigned short server_port,
				     fawkes::Logger *logger)
  : io_service_(io_service), resolver_(io_service_),
    server_host_(server_host), server_port_(server_port), logger_(logger),
    client_socket(io_service_), server_socket(io_service_)
{
}


/** Destruct mapping.
 * This closes both, client and server sockets. This destructor
 * assumes that the io_service has been cancelled.
 */
OpenPRSServerProxy::Mapping::~Mapping()
{
  boost::system::error_code err;
  client_socket.shutdown(ip::tcp::socket::shutdown_both, err);
  client_socket.close();
  server_socket.shutdown(ip::tcp::socket::shutdown_both, err);
  server_socket.close();
}


/** A client has connected, start this mapping. */
void
OpenPRSServerProxy::Mapping::start()
{
  logger_->log_info("OPRS-server-proxy", "Client connected, connecting to server");
  ip::tcp::resolver::query query(server_host_, boost::lexical_cast<std::string>(server_port_));
  resolver_.async_resolve(query,
			  boost::bind(&OpenPRSServerProxy::Mapping::handle_resolve, this,
				      boost::asio::placeholders::error,
				      boost::asio::placeholders::iterator));

}


bool
OpenPRSServerProxy::Mapping::alive() const
{
  return client_socket.is_open();
}


void
OpenPRSServerProxy::Mapping::disconnect()
{
  logger_->log_info("OPRS-server-proxy", "Disconnecting %s", client_name.c_str());
  boost::system::error_code ec;
  client_socket.shutdown(ip::tcp::socket::shutdown_both, ec);
  client_socket.close();
}


void
OpenPRSServerProxy::Mapping::handle_resolve(const boost::system::error_code& err,
					    ip::tcp::resolver::iterator endpoint_iterator)
{
  if (! err) {
    // Attempt a connection to each endpoint in the list until we
    // successfully establish a connection.
#if BOOST_ASIO_VERSION > 100409
    boost::asio::async_connect(server_socket, endpoint_iterator,
#else
    server_socket.async_connect(*endpoint_iterator,
#endif
			       boost::bind(&OpenPRSServerProxy::Mapping::handle_connect, this,
					   boost::asio::placeholders::error));
  } else {
    disconnect();
  }
}

void
OpenPRSServerProxy::Mapping::handle_connect(const boost::system::error_code &err)
{
  if (! err) {

    try {
      // forward greeting
      std::string greeting = read_string_from_socket(server_socket);
      logger_->log_info("OPRS-server-proxy", "Forwarding greeting '%s'", greeting.c_str());
      write_string_to_socket(client_socket, greeting);

      int client_pid = 0;
      int client_use_x = 0;

      logger_->log_info("OPRS-server-proxy", "Reading client details");
      // now read connection details
      client_name  = read_string_from_socket(client_socket);
      client_pid   = read_int_from_socket(client_socket);
      client_use_x = read_int_from_socket(client_socket);

      logger_->log_info("OPRS-server-proxy", "Got client info: %s %i %s",
			client_name.c_str(), client_pid, client_use_x ? "XOPRS" : "OPRS");

      // forward to server
      write_string_to_socket(server_socket, client_name);
      write_int_to_socket(server_socket, client_pid);
      write_int_to_socket(server_socket, client_use_x);

      start_recv_client();
      start_recv_server();
    } catch (Exception &e) {
      disconnect();
    }
  } else {
    disconnect();
  }
}


void
OpenPRSServerProxy::Mapping::start_recv_client()
{
  boost::asio::async_read(client_socket,
			  boost::asio::buffer(&client_in_num_completions_, sizeof(client_in_num_completions_)),
			  boost::bind(&OpenPRSServerProxy::Mapping::handle_recv_client,
				      this, boost::asio::placeholders::error));
}

void
OpenPRSServerProxy::Mapping::start_recv_server()
{
  boost::asio::async_read_until(server_socket, server_buffer_, '\n',
				boost::bind(&OpenPRSServerProxy::Mapping::handle_recv_server,
					    this, boost::asio::placeholders::error));
}


void
OpenPRSServerProxy::Mapping::handle_recv_server(const boost::system::error_code &err)
{
  if (! err) {
    std::string line;
    std::istream in_stream(&server_buffer_);
    std::getline(in_stream, line);

    logger_->log_info("OPRS-server-proxy", "Forwarding S->C line '%s'", line.c_str());
    write_string_newline_to_socket(client_socket, line);

    start_recv_server();
  } else {
    disconnect();
  }
}


void
OpenPRSServerProxy::Mapping::handle_recv_client(const boost::system::error_code &err)
{
  if (! err) {
    client_in_num_completions_ = ntohl(client_in_num_completions_);
    for (int i = 0; i < client_in_num_completions_; ++i) {
      std::string c = read_string_from_socket(client_socket);
      write_string_to_socket(server_socket, c);
    }

    start_recv_client();
  } else {
    disconnect();
  }
}


void
OpenPRSServerProxy::Mapping::transmit_command(const std::string &command)
{
  write_string_newline_to_socket(client_socket, command);
}

/** Read an int from a given socket.
 * @param socket socket to read from
 * @return read value
 */
int
OpenPRSServerProxy::read_int_from_socket(boost::asio::ip::tcp::socket &socket)
{
  int32_t value;
  boost::system::error_code ec;
  boost::asio::read(socket, boost::asio::buffer(&value, sizeof(value)), ec);
  if (ec) {
    throw Exception("Failed to read int from socket: %s", ec.message().c_str());
  } else {
    return ntohl(value);
  }
}

/** Read a string from a given socket.
 * @param socket socket to read from
 * @return read value
 */
std::string
OpenPRSServerProxy::read_string_from_socket(boost::asio::ip::tcp::socket &socket)
{
  uint32_t s_size = 0;
  boost::system::error_code ec;
  boost::asio::read(socket, boost::asio::buffer(&s_size, sizeof(s_size)), ec);
  if (ec) {
    throw Exception("Failed to read string size from socket: %s", ec.message().c_str());
  }
  s_size = ntohl(s_size);

  char s[s_size + 1];
  boost::asio::read(socket, boost::asio::buffer(s, s_size), ec);
  if (ec) {
    throw Exception("Failed to read string content from socket: %s", ec.message().c_str());
  }
  s[s_size] = 0;

  return s;
}


/** Write an int to a given socket.
 * @param socket socket to write to
 * @param i value to write
 */
void
OpenPRSServerProxy::write_int_to_socket(boost::asio::ip::tcp::socket &socket, int i)
{
  boost::system::error_code ec;
  int32_t value = htonl(i);
  boost::asio::write(socket, boost::asio::buffer(&value, sizeof(value)), ec);
  if (ec) {
    throw Exception("Failed to write int to socket: %s", ec.message().c_str());
  }
}

/** Write a string to a given socket.
 * @param socket socket to write to
 * @param str string value to write
 */
void
OpenPRSServerProxy::write_string_to_socket(boost::asio::ip::tcp::socket &socket,
					   const std::string &str)
{
  boost::system::error_code ec;
  uint32_t s_size = htonl(str.size());
  std::array<boost::asio::const_buffer, 2> buffers;
  buffers[0] = boost::asio::buffer(&s_size, sizeof(s_size));
  buffers[1] = boost::asio::buffer(str.c_str(), str.size());

  boost::asio::write(socket, buffers, ec);
  if (ec) {
    throw Exception("Failed to write string to socket: %s", ec.message().c_str());
  }
}


/** Write a string followed by a newline character to a given socket.
 * @param socket socket to write to
 * @param str string value to write
 */
void
OpenPRSServerProxy::write_string_newline_to_socket(boost::asio::ip::tcp::socket &socket,
						   const std::string &str)
{
  boost::system::error_code ec;
  std::string s = str + "\n";
  boost::asio::write(socket, boost::asio::buffer(s.c_str(), s.size()), ec);
  if (ec) {
    throw Exception("Failed to write string to socket: %s", ec.message().c_str());
  }
}


} // end namespace fawkes
