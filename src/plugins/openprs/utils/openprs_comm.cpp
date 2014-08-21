
/***************************************************************************
 *  openprs_comm.cpp - OpenPRS communication wrapper for Fawkes
 *
 *  Created: Mon Aug 18 14:55:51 2014
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

#include "openprs_comm.h"
#include "proc.h"
#include "openprs_server_proxy.h"
#include <core/exception.h>

#include <opaque-pub.h>
#include <mp-pub.h>

#include <unistd.h>

// these exist in libExtMP and are exported, but not mentioned in the header
extern "C" {
  void send_message_string_socket(int socket, Symbol rec, PString message );
  void broadcast_message_string_socket(int socket, PString message );
  void multicast_message_string_socket(int socket, unsigned int nb_recs, Symbol *recs, PString message );
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class OpenPRSComm <plugins/openprs/utils/openprs_comm.h>
 * OpenPRS communication wrapper.
 * This class provides communication facilities via the OpenPRS message passer
 * as well as through the OpenPRS server proxy.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param local_name the local name with which the communication wrapper will be
 * registered to the message parser. This can be used to send messages from the
 * kernel.
 * @param hostname host where the message passer runs
 * @param port TCP port where the message passer listens
 * @param server_proxy server proxy to use to send commands to kernels
 */
OpenPRSComm::OpenPRSComm(const char *local_name,
			 const char *hostname, unsigned short port, OpenPRSServerProxy *server_proxy)
  : name_(local_name), server_proxy_(server_proxy)
{
  mp_socket_ = external_register_to_the_mp_host_prot(local_name, hostname, port, STRINGS_PT);
  if (mp_socket_ == -1) {
    throw Exception("Failed to connect to OpenPRS as '%s'", local_name);
  }
}


/** Destructor. */
OpenPRSComm::~OpenPRSComm()
{
  if (mp_socket_ >= 0) {
    ::close(mp_socket_);
  }
}


/** Send a message to an OpenPRS kernel.
 * @param recipient OpenPRS kernel name to send to
 * @param message message to send, cf. OpenPRS manual for valid messages
 */
void
OpenPRSComm::send_message(const char *recipient, const char *message)
{
  send_message_string_socket(mp_socket_, recipient, (char *)message);
}


/** Send a message to all OpenPRS kernels.
 * @param message message to send, cf. OpenPRS manual for valid messages
 */
void
OpenPRSComm::broadcast_message(const char *message)
{
  broadcast_message_string_socket(mp_socket_, (char *)message);
}


/** Send a message to multiple OpenPRS kernel.
 * @param recipients Vector of OpenPRS kernel names to send to
 * @param message message to send, cf. OpenPRS manual for valid messages
 */
void
OpenPRSComm::multicast_message(std::vector<const char *> &recipients, const char *message)
{
  multicast_message_string_socket(mp_socket_, recipients.size(), &(recipients[0]), (char *)message);
}


/** Send a message to an OpenPRS kernel.
 * @param recipient OpenPRS kernel name to send to
 * @param message message to send, cf. OpenPRS manual for valid messages
 */
void
OpenPRSComm::send_message(const std::string &recipient, const std::string &message)
{
  send_message_string_socket(mp_socket_, recipient.c_str(), (char *)message.c_str());
}


/** Send a message to all OpenPRS kernels.
 * @param message message to send, cf. OpenPRS manual for valid messages
 */
void
OpenPRSComm::broadcast_message(const std::string &message)
{
  broadcast_message_string_socket(mp_socket_, (char *)message.c_str());
}


/** Send a message to multiple OpenPRS kernel.
 * @param recipients Vector of OpenPRS kernel names to send to
 * @param message message to send, cf. OpenPRS manual for valid messages
 */
void
OpenPRSComm::multicast_message(const std::vector<std::string> &recipients, const std::string &message)
{
  std::vector<const char *> recs;
  recs.resize(recipients.size());
  for (size_t i = 0; i < recipients.size(); ++i) {
    recs[i] = recipients[i].c_str();
  }
  multicast_message_string_socket(mp_socket_, recs.size(), &(recs[0]), (char *)message.c_str());
}


/** Transmit a command to an OpenPRS kernel.
 * This works equivalent to the transmit oprs-server console command.
 * @param recipient OpenPRS kernel name to send to
 * @param message command to send, cf. OpenPRS manual for valid commands
 */
void
OpenPRSComm::transmit_command(const char *recipient, const char *message)
{
  server_proxy_->transmit_command(recipient, message);
}

/** Transmit a command to an OpenPRS kernel.
 * This works equivalent to the transmit oprs-server console command.
 * @param recipient OpenPRS kernel name to send to
 * @param message command to send, cf. OpenPRS manual for valid commands
 */
void
OpenPRSComm::transmit_command(const std::string &recipient, const std::string &message)
{
  server_proxy_->transmit_command(recipient, message);
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
OpenPRSComm::transmit_command_f(const std::string &recipient, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  server_proxy_->transmit_command_v(recipient, format, arg);
  va_end(arg);
}


} // end namespace fawkes
