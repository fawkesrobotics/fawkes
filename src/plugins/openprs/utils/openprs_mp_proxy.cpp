
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

#include "openprs_mp_proxy.h"

#include <core/exception.h>
#include <logging/logger.h>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost::asio;

// Types copied from OPRS because they are not public there
namespace OPRS {
  typedef enum {MESSAGE_MT = 1, BROADCAST_MT, MULTICAST_MT, DISCONNECT_MT } Message_Type;
  typedef enum {REGISTER_OK, REGISTER_NAME_CONFLICT, REGISTER_DENIED} Register_Type;
  typedef enum {MESSAGES_PT, STRINGS_PT} Protocol_Type;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenPRSMessagePasserProxy "openprs_mp_proxy.h"
 * Proxy for the OpenPRS server communication.
 * Using this proxy allows to inject commands into the communication between
 * oprs-server and oprs (or xoprs).
 * @author Tim Niemueller
 */


/** Constructor.
 * @param tcp_port port to listen on for incoming connections
 * @param mp_host host of mp-oprs to connect to
 * @param mp_port TCP port that mp-oprs listens on
 * @param logger logger for informational messages
 */
OpenPRSMessagePasserProxy::OpenPRSMessagePasserProxy(unsigned short tcp_port,
						     const std::string &mp_host, unsigned short mp_port,
						     fawkes::Logger *logger)
  : io_service_work_(io_service_), acceptor_(io_service_, ip::tcp::endpoint(ip::tcp::v6(), tcp_port)),
    mp_host_(mp_host), mp_port_(mp_port), logger_(logger)
{
  acceptor_.set_option(socket_base::reuse_address(true));
  io_service_thread_ = std::thread([this]() { this->io_service_.run(); });
  start_accept();
}

/** Destructor. */
OpenPRSMessagePasserProxy::~OpenPRSMessagePasserProxy()
{
  io_service_.stop();
  io_service_thread_.join();
}

/** Start accepting connections. */
void
OpenPRSMessagePasserProxy::start_accept()
{
  Mapping::Ptr mapping(new Mapping(io_service_, mp_host_, mp_port_, logger_));
  acceptor_.async_accept(mapping->client_socket,
			 boost::bind(&OpenPRSMessagePasserProxy::handle_accept, this,
				     mapping, boost::asio::placeholders::error));
}

void
OpenPRSMessagePasserProxy::handle_accept(Mapping::Ptr mapping,
					 const boost::system::error_code& error)
{
  if (! error) {
    mappings_.push_back(mapping);
    mapping->start();
  }

  start_accept();
}


OpenPRSMessagePasserProxy::Mapping::Mapping(boost::asio::io_service &io_service,
					    const std::string &mp_host, unsigned short mp_port,
					    fawkes::Logger *logger)
  : io_service_(io_service), resolver_(io_service_),
    server_host_(mp_host), server_port_(mp_port), logger_(logger),
    client_socket(io_service_), server_socket(io_service_)
{
}


/** Destruct mapping.
 * This closes both, client and server sockets. This destructor
 * assumes that the io_service has been cancelled.
 */
OpenPRSMessagePasserProxy::Mapping::~Mapping()
{
  boost::system::error_code err;
  client_socket.shutdown(ip::tcp::socket::shutdown_both, err);
  client_socket.close();
  server_socket.shutdown(ip::tcp::socket::shutdown_both, err);
  server_socket.close();
}


/** A client has connected, start this mapping. */
void
OpenPRSMessagePasserProxy::Mapping::start()
{
  client_prot = read_int_from_socket(client_socket);
  client_name = read_string_from_socket(client_socket);

  logger_->log_info("OPRS-mp-proxy", "Client %s connected", client_name.c_str());

  ip::tcp::resolver::query query(server_host_, boost::lexical_cast<std::string>(server_port_));
  resolver_.async_resolve(query,
			  boost::bind(&OpenPRSMessagePasserProxy::Mapping::handle_resolve, this,
				      boost::asio::placeholders::error,
				      boost::asio::placeholders::iterator));

}


bool
OpenPRSMessagePasserProxy::Mapping::alive() const
{
  return client_socket.is_open();
}


/** Disconnect this client. */
void
OpenPRSMessagePasserProxy::Mapping::disconnect()
{
  disconnect("disconnect", "API call");
}


void
OpenPRSMessagePasserProxy::Mapping::disconnect(const char *where, const char *reason)
{
  logger_->log_warn("OPRS-mp-proxy", "Client %s disconnected (%s: %s)",
		    client_name.c_str(), where, reason);
  boost::system::error_code ec;
  client_socket.shutdown(ip::tcp::socket::shutdown_both, ec);
  client_socket.close();
}


void
OpenPRSMessagePasserProxy::Mapping::handle_resolve(const boost::system::error_code& err,
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
			       boost::bind(&OpenPRSMessagePasserProxy::Mapping::handle_connect, this,
					   boost::asio::placeholders::error));
  } else {
    disconnect("handle_resolve", err.message().c_str());
  }
}

void
OpenPRSMessagePasserProxy::Mapping::handle_connect(const boost::system::error_code &err)
{
  if (! err) {
    write_int_to_socket(server_socket, client_prot);
    write_string_to_socket(server_socket, client_name);

    // asynchronously read registration reply
    boost::asio::async_read(server_socket,
			    boost::asio::buffer(&server_in_reg_reply_, sizeof(server_in_reg_reply_)),
			    boost::bind(&OpenPRSMessagePasserProxy::Mapping::handle_recv_server_reg_reply,
					this, boost::asio::placeholders::error));

  } else {
    disconnect("handle_connect", err.message().c_str());
  }
}


void
OpenPRSMessagePasserProxy::Mapping::handle_recv_server_reg_reply(const boost::system::error_code &err)
{
  write_int_to_socket(client_socket, server_in_reg_reply_);

  if (server_in_reg_reply_ == OPRS::REGISTER_OK) {
    start_recv_client();
    start_recv_server();
  } else {
    disconnect("recv_server_reg_reply", err.message().c_str());
  }
}


void
OpenPRSMessagePasserProxy::Mapping::start_recv_client()
{
  boost::asio::async_read(client_socket,
			  boost::asio::buffer(&client_in_msg_type_, sizeof(client_in_msg_type_)),
			  boost::bind(&OpenPRSMessagePasserProxy::Mapping::handle_recv_client,
				      this, boost::asio::placeholders::error));
}


void
OpenPRSMessagePasserProxy::Mapping::handle_recv_client(const boost::system::error_code &err)
{
  if (! err) {
    try {
      std::vector<std::string> multicast_recipients;
      std::string message;
      std::string recipient;

      client_in_msg_type_ = ntohl(client_in_msg_type_);

      switch (client_in_msg_type_) {
      case OPRS::DISCONNECT_MT:
	logger_->log_info("OPRS-mp-proxy", "Disconnecting %s", client_name.c_str());
	disconnect("recv_client", "Client disconnected");
	return;

      case OPRS::MESSAGE_MT:
	recipient = read_string_from_socket(client_socket);
	break;

      case OPRS::MULTICAST_MT:
	multicast_recipients.resize(read_int_from_socket(client_socket));
	break;

      case OPRS::BROADCAST_MT: break; // nothing to do here

      default:
	disconnect("recv_client", "Unknown message type");
	return;
      }
    
      message = read_string_from_socket(client_socket);

      if (client_in_msg_type_ == OPRS::MULTICAST_MT) {
	for (size_t i = 0; i < multicast_recipients.size(); ++i) {
	  multicast_recipients[i] = read_string_from_socket(client_socket);
	}
      }

      // debug output
      switch (client_in_msg_type_) {
      case OPRS::MESSAGE_MT:
	logger_->log_info("OPRS-mp-proxy", "Forwarding unicast %s->%s: '%s'",
			  client_name.c_str(), recipient.c_str(), message.c_str());
	break;

      case OPRS::MULTICAST_MT:
	{
	  std::string recipients;
	  for (size_t i = 0; i < multicast_recipients.size(); ++i) {
	    if (i > 0)  recipients += ", ";
	    recipients += multicast_recipients[i];
	  }

	  logger_->log_info("OPRS-mp-proxy", "Forwarding multicast %s->(%s): '%s'",
			    client_name.c_str(), recipients.c_str(), message.c_str());
	}
	break;

      case OPRS::BROADCAST_MT:
	logger_->log_info("OPRS-mp-proxy", "Forwarding broadcast %s->*: '%s'",
			  client_name.c_str(), message.c_str());
	break;

      default: break;
      }

      // now re-send message to server
      write_int_to_socket(server_socket, client_in_msg_type_);

      switch (client_in_msg_type_) {
      case OPRS::MESSAGE_MT:
	write_string_to_socket(server_socket, recipient);
	write_string_to_socket(server_socket, message);
	break;

      case OPRS::MULTICAST_MT:
	write_string_to_socket(server_socket, message);
	for (size_t i = 0; i < multicast_recipients.size(); ++i) {
	  write_string_to_socket(server_socket, multicast_recipients[i]);
	}
	break;

      case OPRS::BROADCAST_MT: // nothing to do here
	write_string_to_socket(server_socket, message);
	break;

      default: break; // cannot happen here anymore
      }

      start_recv_client();
    } catch (Exception &e) {
      disconnect("recv_client", e.what_no_backtrace());
    }
  } else {
    disconnect("recv_client", err.message().c_str());
  }
}


void
OpenPRSMessagePasserProxy::Mapping::start_recv_server()
{
  if (client_prot == OPRS::MESSAGES_PT) {
    logger_->log_warn("OPRS-mp-proxy", "Starting listening for %s in MESSAGES_PT mode", client_name.c_str());
    boost::asio::async_read_until(server_socket, server_buffer_, '\n',
				  boost::bind(&OpenPRSMessagePasserProxy::Mapping::handle_recv_server_message_pt,
					      this, boost::asio::placeholders::error));
  } else {
    // tried async_read_some with null buffers but always immediately fires without data available
    logger_->log_warn("OPRS-mp-proxy", "Starting listening for %s in STRINGS_PT mode", client_name.c_str());
    server_socket.async_read_some(boost::asio::null_buffers(),
				  boost::bind(&OpenPRSMessagePasserProxy::Mapping::handle_recv_server_strings_pt,
					      this, boost::asio::placeholders::error));
  }
}


void
OpenPRSMessagePasserProxy::Mapping::handle_recv_server_message_pt(const boost::system::error_code &err)
{
  if (! err) {
    std::string line;
    std::istream in_stream(&server_buffer_);
    std::getline(in_stream, line);

    logger_->log_info("OPRS-mp-proxy", "Forwarding server ->%s: '%s\\n'",
		      client_name.c_str(), line.c_str());

    // resend to client
    write_string_newline_to_socket(client_socket, line);

    start_recv_server();
  } else {
    disconnect("recv_server_message_pt", err.message().c_str());
  }
}


void
OpenPRSMessagePasserProxy::Mapping::handle_recv_server_strings_pt(const boost::system::error_code &err)
{
  if (! err) {
    try {
      std::string sender  = read_string_from_socket(server_socket);
      std::string message = read_string_from_socket(server_socket);

      logger_->log_info("OPRS-mp-proxy", "Forwarding server %s->%s: '%s'",
			sender.c_str(), client_name.c_str(), message.c_str());

      // resend to client
      write_string_to_socket(client_socket, sender);
      write_string_to_socket(client_socket, message);

      start_recv_server();
    } catch (Exception &e) {
      disconnect("recv_server_strings_pt", e.what_no_backtrace());
    }
  } else {
    disconnect("recv_server_strings_pt", err.message().c_str());
  }
}


int
OpenPRSMessagePasserProxy::Mapping::read_int_from_socket(boost::asio::ip::tcp::socket &socket)
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

std::string
OpenPRSMessagePasserProxy::Mapping::read_string_from_socket(boost::asio::ip::tcp::socket &socket)
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


void
OpenPRSMessagePasserProxy::Mapping::write_int_to_socket(boost::asio::ip::tcp::socket &socket, int i)
{
  boost::system::error_code ec;
  int32_t value = htonl(i);
  boost::asio::write(socket, boost::asio::buffer(&value, sizeof(value)), ec);
  if (ec) {
    throw Exception("Failed to write int to socket: %s", ec.message().c_str());
  }
}


void
OpenPRSMessagePasserProxy::Mapping::write_string_to_socket(boost::asio::ip::tcp::socket &socket, std::string &str)
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


void
OpenPRSMessagePasserProxy::Mapping::write_string_newline_to_socket(boost::asio::ip::tcp::socket &socket,
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
