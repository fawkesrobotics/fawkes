
/***************************************************************************
 *  network_logger.cpp - Fawkes network logger
 *
 *  Created: Sat Dec 15 00:48:52 2007 (after I5 xmas party)
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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

#include <network_logger/network_logger.h>

#include <core/threading/mutex.h>
#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>
#include <netcomm/utils/ntoh64.h>

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


/** @class NetworkLogger <network_logger/network_logger.h>
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
			    const char *component, bool is_exception,
			    const char *format, va_list va)
{
  struct timeval now;
  if ( t == NULL ) {
    gettimeofday(&now, NULL);
    t = &now;
  }

  NetworkLoggerMessageContent *content = new NetworkLoggerMessageContent(level, t,
									 component,
									 is_exception,
									 format, va);

  for ( ssit_ = subscribers_.begin(); ssit_ != subscribers_.end(); ++ssit_) {
    NetworkLoggerMessageContent *content_copy = new NetworkLoggerMessageContent(content);
    try {
      hub->send(*ssit_, FAWKES_CID_NETWORKLOGGER, MSGTYPE_LOGMESSAGE, content_copy);
    } catch (Exception &e) {
      // Boom, can't do anything about it, logging could cause infinite loop...
    }
  }
  
  delete content;
}


void
NetworkLogger::send_message(Logger::LogLevel level, struct timeval *t,
			    const char *component, bool is_exception,
			    const char *message)
{
  struct timeval now;
  if ( t == NULL ) {
    gettimeofday(&now, NULL);
    t = &now;
  }

  NetworkLoggerMessageContent *content = new NetworkLoggerMessageContent(level, t,
									 component,
									 is_exception,
									 message);
  
  for ( ssit_ = subscribers_.begin(); ssit_ != subscribers_.end(); ++ssit_) {
    NetworkLoggerMessageContent *content_copy = new NetworkLoggerMessageContent(content);
    try {
      hub->send(*ssit_, FAWKES_CID_NETWORKLOGGER, MSGTYPE_LOGMESSAGE, content_copy);
    } catch (Exception &e) {
      // Boom, can't do anything about it, logging could cause infinite loop...
    }
  }

  delete content;
}


void
NetworkLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_DEBUG) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    send_message(LL_DEBUG, NULL, component, /* exception? */ false, format, va);
    subscribers_.unlock();
  }
}


void
NetworkLogger::vlog_info(const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_INFO) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    send_message(LL_INFO, NULL, component, /* exception? */ false, format, va);
   subscribers_.unlock();
  }
}


void
NetworkLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_WARN) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    send_message(LL_WARN, NULL, component, /* exception? */ false, format, va);
   subscribers_.unlock();
  }
}


void
NetworkLogger::vlog_error(const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_ERROR) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    send_message(LL_ERROR, NULL, component, /* exception? */ false, format, va);
   subscribers_.unlock();
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
  if ((log_level <= LL_DEBUG) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_DEBUG, NULL, component, /* exception? */ true, *i);
    } 
   subscribers_.unlock();
  }
}

void
NetworkLogger::log_info(const char *component, Exception &e)
{
  if ((log_level <= LL_INFO) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_INFO, NULL, component, /* exception? */ true, *i);
    } 
   subscribers_.unlock();
  }
}


void
NetworkLogger::log_warn(const char *component, Exception &e)
{
  if ((log_level <= LL_WARN) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_WARN, NULL, component, /* exception? */ true, *i);
    } 
   subscribers_.unlock();
  }
}


void
NetworkLogger::log_error(const char *component, Exception &e)
{
  if ((log_level <= LL_ERROR) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_ERROR, NULL, component, /* exception? */ true, *i);
    }
   subscribers_.unlock();
  }
}




void
NetworkLogger::vtlog_debug(struct timeval *t, const char *component,
			   const char *format, va_list va)
{
  if ((log_level <= LL_DEBUG) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    send_message(LL_DEBUG, t, component, /* exception? */ false, format, va);
    subscribers_.unlock();
  }
}


void
NetworkLogger::vtlog_info(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_INFO) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    send_message(LL_INFO, t, component, /* exception? */ false, format, va);
   subscribers_.unlock();
  }
}


void
NetworkLogger::vtlog_warn(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_WARN) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    send_message(LL_WARN, t, component, /* exception? */ false, format, va);
   subscribers_.unlock();
  }
}


void
NetworkLogger::vtlog_error(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ((log_level <= LL_ERROR) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    send_message(LL_ERROR, t, component, /* exception? */ false, format, va);
   subscribers_.unlock();
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
  if ((log_level <= LL_DEBUG) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_DEBUG, t, component, /* exception? */ true, *i);
    } 
   subscribers_.unlock();
  }
}

void
NetworkLogger::tlog_info(struct timeval *t, const char *component, Exception &e)
{
  if ((log_level <= LL_INFO) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_INFO, t, component, /* exception? */ true, *i);
    } 
   subscribers_.unlock();
  }
}


void
NetworkLogger::tlog_warn(struct timeval *t, const char *component, Exception &e)
{
  if ((log_level <= LL_WARN) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_WARN, t, component, /* exception? */ true, *i);
    } 
   subscribers_.unlock();
  }
}


void
NetworkLogger::tlog_error(struct timeval *t, const char *component, Exception &e)
{
  if ((log_level <= LL_ERROR) && (! subscribers_.empty()) ) {
    subscribers_.lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      send_message(LL_ERROR, t, component, /* exception? */ true, *i);
    } 
   subscribers_.unlock();
  }
}


void
NetworkLogger::handle_network_message(FawkesNetworkMessage *msg)
{
  if ( (msg->cid() == FAWKES_CID_NETWORKLOGGER) &&
       (msg->msgid() == MSGTYPE_SUBSCRIBE) ) {
    subscribers_.lock();
    subscribers_.push_back(msg->clid());
    subscribers_.sort();
    subscribers_.unique();
    subscribers_.unlock();
  }
}

void
NetworkLogger::client_connected(unsigned int clid)
{
}


void
NetworkLogger::client_disconnected(unsigned int clid)
{
  subscribers_.remove_locked(clid);
}


/** @class NetworkLoggerMessageContent <network_logger/network_logger.h>
 * Message sent over the network with a log message.
 * Contains a buffer with a small header and two null-terminated strings, the first
 * being the component and the second being the real message.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param log_level Log level
 * @param t time
 * @param component component string
 * @param is_exception true if this message originates from an exception, false otherwise
 * @param format message string format
 * @param va va_list containing the arguments for the given format
 */
NetworkLoggerMessageContent::NetworkLoggerMessageContent(Logger::LogLevel log_level,
							 struct timeval *t,
							 const char *component,
							 bool is_exception,
							 const char *format, va_list va)
{
  char *tmp = NULL;
  int tmplen;
  if ( (tmplen = vasprintf(&tmp, format, va)) != -1 ) {
    _payload_size = sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + tmplen + 2;
    _payload = calloc(1, _payload_size);
    own_payload_ = true;
    header = (NetworkLogger::network_logger_header_t *)_payload;
    header->log_level    = log_level;
    header->exception    = is_exception ? 1 : 0;
    header->time_sec     = hton64(t->tv_sec);
    header->time_usec    = htonl(t->tv_usec);
    copy_payload(sizeof(NetworkLogger::network_logger_header_t), component, strlen(component));
    copy_payload(sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + 1, tmp, tmplen);
    component_ = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t);
    message_   = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + 1;
    free(tmp);
  }
}


/** Constructor.
 * @param log_level Log level
 * @param t time
 * @param component component string
 * @param is_exception true if this message originates from an exception, false otherwise
 * @param message message string.
 */
NetworkLoggerMessageContent::NetworkLoggerMessageContent(Logger::LogLevel log_level,
							 struct timeval *t,
							 const char *component,
							 bool is_exception,
							 const char *message)
{
  _payload_size = sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + strlen(message) + 2;
  _payload = calloc(1, _payload_size);
  own_payload_ = true;
  header = (NetworkLogger::network_logger_header_t *)_payload;
  header->log_level    = log_level;
  header->exception    = is_exception ? 1 : 0;
  header->time_sec     = hton64(t->tv_sec);
  header->time_usec    = htonl(t->tv_usec);
  copy_payload(sizeof(NetworkLogger::network_logger_header_t), component, strlen(component));
  copy_payload(sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + 1, message, strlen(message));
  component_ = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t);
  message_   = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t) + strlen(component) + 1;
}


/** Copy constructor.
 * @param content content to copy
 */
NetworkLoggerMessageContent::NetworkLoggerMessageContent(const NetworkLoggerMessageContent *content)
{
  _payload_size = content->_payload_size;
  _payload = malloc(_payload_size);
  own_payload_ = true;
  memcpy(_payload, content->_payload, _payload_size);
  header = (NetworkLogger::network_logger_header_t *)_payload;
  component_ = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t);
  message_   = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t) + strlen(component_) + 1;
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
  own_payload_ = false;
  header = (NetworkLogger::network_logger_header_t *)_payload;
  component_ = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t);
  message_   = (char *)_payload + sizeof(NetworkLogger::network_logger_header_t) + strlen(component_) + 1;
}

/** Destructor. */
NetworkLoggerMessageContent::~NetworkLoggerMessageContent()
{
  if (own_payload_)  free(_payload);
}

/** Get time.
 * @return time of the log message
 */
struct timeval
NetworkLoggerMessageContent::get_time() const
{
  struct timeval rv;
  rv.tv_sec  = (time_t)ntoh64(header->time_sec);
  rv.tv_usec = ntohl(header->time_usec);
  return rv;
}


/** Get component.
 * @return component string
 */
const char *
NetworkLoggerMessageContent::get_component() const
{
  return component_;
}


/** Get message.
 * @return message string
 */
const char *
NetworkLoggerMessageContent::get_message() const
{
  return message_;
}


/** Log level.
 * @return log level.
 */
Logger::LogLevel
NetworkLoggerMessageContent::get_loglevel() const
{
  return (Logger::LogLevel)header->log_level;
}


/** Check if message was generated by exception.
 * @return true if message was generated by exception, false otherwise
 */
bool
NetworkLoggerMessageContent::is_exception() const
{
  return (header->exception == 1);
}

} // end namespace fawkes
