
/***************************************************************************
 *  network_logger.cpp - Fawkes network logger
 *
 *  Created: Sat Dec 15 00:48:52 2007 (after I5 xmas party)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
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

#include <netcomm/utils/network_logger.h>

#include <core/threading/mutex.h>
#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>

#include <sys/time.h>
#include <time.h>
#include <netinet/in.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>

namespace fawkes {

void
NetworkLoggerMessageContent::serialize()
{
}


/** @class NetworkLogger <netcomm/utils/network_logger.h>
 * Interface for logging to network clients.
 * The NetwokLogger will pipe all output to clients that subscribed for log
 * messages.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param hub FawkesNetworkHub to use to send and receive messages
 * @param log_level minimum level to log
 */
NetworkLogger::NetworkLogger(FawkesNetworkHub *hub, LogLevel log_level)
  : Logger(log_level),
    FawkesNetworkHandler(FAWKES_CID_NETWORKLOGGER)
{
  this->hub = hub;

  hub->add_handler(this);
}


/** Destructor. */
NetworkLogger::~NetworkLogger()
{
  hub->remove_handler(this);
}


void
NetworkLogger::send_message(Logger::LogLevel level, struct timeval *t,
			    const char *component, const char *format, va_list va)
{
  struct timeval now;
  if ( t == NULL ) {
    gettimeofday(&now, NULL);
    t = &now;
  }

  NetworkLoggerMessageContent *content = new NetworkLoggerMessageContent(level, t,
									 component,
									 format, va);
  
  for ( __ssit = __subscribers.begin(); __ssit != __subscribers.end(); ++__ssit) {
    NetworkLoggerMessageContent *content_copy = new NetworkLoggerMessageContent(content);
    hub->send(*__ssit, FAWKES_CID_NETWORKLOGGER, MSGTYPE_LOGMESSAGE, content_copy);
  }
  
  delete content;
}


void
NetworkLogger::send_message(Logger::LogLevel level, struct timeval *t,
			    const char *component, const char *message)
{
  struct timeval now;
  if ( t == NULL ) {
    gettimeofday(&now, NULL);
    t = &now;
  }

  NetworkLoggerMessageContent *content = new NetworkLoggerMessageContent(LL_DEBUG, t,
									 component,
									 message);
  
  for ( __ssit = __subscribers.begin(); __ssit != __subscribers.end(); ++__ssit) {
    NetworkLoggerMessageContent *content_copy = new NetworkLoggerMessageContent(content);
    hub->send(*__ssit, FAWKES_CID_NETWORKLOGGER, MSGTYPE_LOGMESSAGE, content_copy);
  }

  delete content;
}


void
NetworkLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_DEBUG) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    send_message(LL_DEBUG, NULL, component, format, va);
    __subscribers.unlock();
  }
}


void
NetworkLogger::vlog_info(const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_INFO) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    send_message(LL_INFO, NULL, component, format, va);
   __subscribers.unlock();
  }
}


void
NetworkLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_WARN) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    send_message(LL_WARN, NULL, component, format, va);
   __subscribers.unlock();
  }
}


void
NetworkLogger::vlog_error(const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_ERROR) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    send_message(LL_ERROR, NULL, component, format, va);
   __subscribers.unlock();
  }
}


void
NetworkLogger::log_debug(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_debug(component, format, arg);
  va_end(arg);
}


void
NetworkLogger::log_info(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_info(component, format, arg);
  va_end(arg);
}


void
NetworkLogger::log_warn(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_warn(component, format, arg);
  va_end(arg);
}


void
NetworkLogger::log_error(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_error(component, format, arg);
  va_end(arg);
}


void
NetworkLogger::log_debug(const char *component, Exception &e)
{
  if ((log_level <= LL_DEBUG) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_DEBUG, NULL, component, *i);
    } 
   __subscribers.unlock();
  }
}

void
NetworkLogger::log_info(const char *component, Exception &e)
{
  if ((log_level <= LL_INFO) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_INFO, NULL, component, *i);
    } 
   __subscribers.unlock();
  }
}


void
NetworkLogger::log_warn(const char *component, Exception &e)
{
  if ((log_level <= LL_WARN) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_WARN, NULL, component, *i);
    } 
   __subscribers.unlock();
  }
}


void
NetworkLogger::log_error(const char *component, Exception &e)
{
  if ((log_level <= LL_ERROR) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_ERROR, NULL, component, *i);
    } 
   __subscribers.unlock();
  }
}




void
NetworkLogger::vtlog_debug(struct timeval *t, const char *component,
			   const char *format, va_list va)
{
  if ((log_level <= LL_DEBUG) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    send_message(LL_DEBUG, t, component, format, va);
    __subscribers.unlock();
  }
}


void
NetworkLogger::vtlog_info(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_INFO) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    send_message(LL_INFO, t, component, format, va);
   __subscribers.unlock();
  }
}


void
NetworkLogger::vtlog_warn(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_WARN) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    send_message(LL_WARN, t, component, format, va);
   __subscribers.unlock();
  }
}


void
NetworkLogger::vtlog_error(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_ERROR) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    send_message(LL_ERROR, t, component, format, va);
   __subscribers.unlock();
  }
}


void
NetworkLogger::tlog_debug(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_debug(t, component, format, arg);
  va_end(arg);
}


void
NetworkLogger::tlog_info(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_info(t, component, format, arg);
  va_end(arg);
}


void
NetworkLogger::tlog_warn(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_warn(t, component, format, arg);
  va_end(arg);
}


void
NetworkLogger::tlog_error(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_error(t, component, format, arg);
  va_end(arg);
}


void
NetworkLogger::tlog_debug(struct timeval *t, const char *component, Exception &e)
{
  if ((log_level <= LL_DEBUG) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_DEBUG, t, component, *i);
    } 
   __subscribers.unlock();
  }
}

void
NetworkLogger::tlog_info(struct timeval *t, const char *component, Exception &e)
{
  if ((log_level <= LL_INFO) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_INFO, t, component, *i);
    } 
   __subscribers.unlock();
  }
}


void
NetworkLogger::tlog_warn(struct timeval *t, const char *component, Exception &e)
{
  if ((log_level <= LL_WARN) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_WARN, t, component, *i);
    } 
   __subscribers.unlock();
  }
}


void
NetworkLogger::tlog_error(struct timeval *t, const char *component, Exception &e)
{
  if ((log_level <= LL_ERROR) && (! __subscribers.empty()) ) {
    __subscribers.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_ERROR, t, component, *i);
    } 
   __subscribers.unlock();
  }
}


void
NetworkLogger::handle_network_message(FawkesNetworkMessage *msg)
{
  if ( (msg->cid() == FAWKES_CID_NETWORKLOGGER) &&
       (msg->msgid() == MSGTYPE_SUBSCRIBE) ) {
    __subscribers.lock();
    __subscribers.push_back(msg->clid());
    __subscribers.sort();
    __subscribers.unique();
    __subscribers.unlock();
  }
}

void
NetworkLogger::client_connected(unsigned int clid)
{
}


void
NetworkLogger::client_disconnected(unsigned int clid)
{
  __subscribers.remove_locked(clid);
}


/** @class NetworkLoggerMessageContent <netcomm/utils/network_logger.h>
 * Message sent over the network with a log message.
 * Contains a buffer with a small header and two null-terminated strings, the first
 * being the component and the second being the real message.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param log_level Log level
 * @param t time
 * @param component component string
 * @param format message string format
 * @param va va_list containing the arguments for the given format
 */
NetworkLoggerMessageContent::NetworkLoggerMessageContent(Logger::LogLevel log_level,
							 struct timeval *t,
							 const char *component,
							 const char *format, va_list va)
{
  char *tmp = NULL;
  int tmplen;
  if ( (tmplen = vasprintf(&tmp, format, va)) != -1 ) {
    _payload_size = sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + tmplen + 2;
    _payload = calloc(1, _payload_size);
    header = (NetworkLogger::network_logger_header_t *)_payload;
    header->log_level    = log_level;
    header->time.tv_sec  = htonl(t->tv_sec);
    header->time.tv_usec = htonl(t->tv_usec);
    copy_payload(sizeof(NetworkLogger::network_logger_header_t), component, strlen(component));
    copy_payload(sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + 1, tmp, tmplen);
    __component = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t);
    __message   = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + 1;
  }
}


/** Constructor.
 * @param log_level Log level
 * @param t time
 * @param component component string
 * @param message message string.
 */
NetworkLoggerMessageContent::NetworkLoggerMessageContent(Logger::LogLevel log_level,
							 struct timeval *t,
							 const char *component,
							 const char *message)
{
  _payload_size = sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + strlen(message) + 2;
  _payload = calloc(1, _payload_size);
  header = (NetworkLogger::network_logger_header_t *)_payload;
  header->log_level    = log_level;
  header->time.tv_sec  = htonl(t->tv_sec);
  header->time.tv_usec = htonl(t->tv_usec);
  copy_payload(sizeof(NetworkLogger::network_logger_header_t), component, strlen(component));
  copy_payload(sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + 1, message, strlen(message));
  __component = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t);
  __message   = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + 1;
}


/** Copy constructor.
 * @param content content to copy
 */
NetworkLoggerMessageContent::NetworkLoggerMessageContent(const NetworkLoggerMessageContent *content)
{
  _payload_size = content->_payload_size;
  _payload = malloc(_payload_size);
  memcpy(_payload, content->_payload, _payload_size);
  header = (NetworkLogger::network_logger_header_t *)_payload;
  __component = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t);
  __message   = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t) + strlen(__component) + 1;
}


/** Message parsing constructor.
 * To be used with FawkesNetworkMessage::msgc().
 * @param component_id component ID
 * @param msg_id message ID
 * @param payload payload
 * @param payload_size payload size
 */
NetworkLoggerMessageContent::NetworkLoggerMessageContent(unsigned int component_id,
							 unsigned int msg_id,
							 void *payload, size_t payload_size)
{
  if ( component_id != FAWKES_CID_NETWORKLOGGER ) {
    throw TypeMismatchException("Wrong CID, expected FAWKES_CID_NETWORKLOGGER");
  }

  _payload = payload;
  _payload_size = payload_size;
  header = (NetworkLogger::network_logger_header_t *)_payload;
  __component = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t);
  __message   = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t) + strlen(__component) + 1;
}

/** Destructor. */
NetworkLoggerMessageContent::~NetworkLoggerMessageContent()
{
}

/** Get time.
 * @return time of the log message
 */
struct timeval
NetworkLoggerMessageContent::get_time() const
{
  struct timeval rv;
  rv.tv_sec  = ntohl(header->time.tv_sec);
  rv.tv_usec = ntohl(header->time.tv_usec);
  return rv;
}


/** Get component.
 * @return component string
 */
const char *
NetworkLoggerMessageContent::get_component() const
{
  return __component;
}


/** Get message.
 * @return message string
 */
const char *
NetworkLoggerMessageContent::get_message() const
{
  return __message;
}


/** Log level.
 * @return log level.
 */
Logger::LogLevel
NetworkLoggerMessageContent::get_loglevel() const
{
  return (Logger::LogLevel)header->log_level;
}

} // end namespace fawkes
