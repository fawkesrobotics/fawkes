
/***************************************************************************
 *  sick_tim55x_ethernet_aqt.cpp - Retrieve data from Sick TiM55x via Ethernet
 *
 *  Created: Sun Jun 15 20:45:42 2014
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

#include "sick_tim55x_ethernet_aqt.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/misc/string_split.h>
#include <utils/math/angle.h>

#include <boost/lexical_cast.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#if BOOST_VERSION < 104800
#  include <boost/bind.hpp>
#endif
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>

using namespace fawkes;

#define RECONNECT_INTERVAL   1000
#define RECEIVE_TIMEOUT      500

/** @class SickTiM55xEthernetAcquisitionThread "sick_tim55x_ethernet_aqt.h"
 * Laser acqusition thread for Sick TiM55x laser range finders.
 * This thread fetches the data from the laser.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 */
SickTiM55xEthernetAcquisitionThread::SickTiM55xEthernetAcquisitionThread(std::string &cfg_name,
							       std::string &cfg_prefix)
  : SickTiM55xCommonAcquisitionThread(cfg_name, cfg_prefix),
    socket_(io_service_), deadline_(io_service_), soft_deadline_(io_service_)
{
  set_name("SickTiM55x(%s)", cfg_name.c_str());
}

void
SickTiM55xEthernetAcquisitionThread::init()
{
  read_common_config();

  cfg_host_ = config->get_string((cfg_prefix_ + "host").c_str());
  cfg_port_ = config->get_string((cfg_prefix_ + "port").c_str());

  socket_mutex_ = new Mutex();

  deadline_.expires_at(boost::posix_time::pos_infin);
  check_deadline();

  soft_deadline_.expires_at(boost::posix_time::pos_infin);
  check_soft_timeout();

  init_device();

  pre_init(config, logger);
}


void
SickTiM55xEthernetAcquisitionThread::finalize()
{
  free(_distances);
  _distances = NULL;

  free(_echoes);
  _echoes = NULL;

  delete socket_mutex_;
}


void
SickTiM55xEthernetAcquisitionThread::loop()
{
  if (socket_.is_open()) {
    try {
      deadline_.expires_from_now(boost::posix_time::milliseconds(RECEIVE_TIMEOUT));

      ec_ = boost::asio::error::would_block;
      bytes_read_ = 0;

      boost::asio::async_read_until(socket_, input_buffer_, '\03',
#if BOOST_VERSION >= 104800
				    (boost::lambda::var(ec_) = boost::lambda::_1,
				     boost::lambda::var(bytes_read_) = boost::lambda::_2));
#else
				    boost::bind(
				      &SickTiM55xEthernetAcquisitionThread::handle_read,
				      this,
				      boost::asio::placeholders::error,
				      boost::asio::placeholders::bytes_transferred
				    ));
#endif

      do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

      reset_distances();
      reset_echoes();

      if (ec_) {
	if (ec_.value() == boost::system::errc::operation_canceled) {
	  logger->log_error(name(), "Data timeout, will try to reconnect");
	} else {
	  logger->log_warn(name(), "Data read error: %s\n", ec_.message().c_str());
	}
	_data_mutex->lock();
	_timestamp->stamp();
	_new_data = true;
	_data_mutex->unlock();
	close_device();

      } else {
	deadline_.expires_at(boost::posix_time::pos_infin);

	unsigned char recv_buf[bytes_read_];
	std::istream in_stream(&input_buffer_);
	in_stream.read((char *)recv_buf, bytes_read_);

	if (bytes_read_ > 0) {
	  try {
	    parse_datagram(recv_buf, bytes_read_);
	  } catch (Exception &e) {
	    logger->log_warn(name(), "Failed to parse datagram, resyncing, exception follows");
	    logger->log_warn(name(), e);
	    resync();
	  }
	}
      }
    } catch (boost::system::system_error &e) {
      if (e.code() == boost::asio::error::eof) {
	close_device();
	logger->log_warn(name(),
			 "Sick TiM55x/Ethernet connection lost, trying to reconnect");
      } else {
	logger->log_warn(name(), "Sick TiM55x/Ethernet failed read: %s", e.what());
      }
    }
  } else {
    try {
      init_device();
      logger->log_warn(name(), "Reconnected to device");
    } catch (Exception &e) {
      // ignore, keep trying
      usleep(RECONNECT_INTERVAL * 1000);
    }
  }

  yield();
}


void
SickTiM55xEthernetAcquisitionThread::open_device()
{
  try {
    boost::asio::ip::tcp::resolver resolver(io_service_);
    boost::asio::ip::tcp::resolver::query
      query(cfg_host_, cfg_port_);
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

    // this is just the overly complicated way to get a timeout on
    // a synchronous connect, cf.
    // http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/example/cpp03/timeouts/blocking_tcp_client.cpp

    deadline_.expires_from_now(boost::posix_time::seconds(5));

    boost::system::error_code ec;

    for (; iter != boost::asio::ip::tcp::resolver::iterator(); ++iter) {
      socket_.close();
      ec_ = boost::asio::error::would_block;
#if BOOST_VERSION >= 104800
      socket_.async_connect(iter->endpoint(), boost::lambda::var(ec_) = boost::lambda::_1);
#else
      socket_.async_connect(iter->endpoint(),
			    boost::bind(&SickTiM55xEthernetAcquisitionThread::handle_read, this,
					boost::asio::placeholders::error, 0));
#endif

      // Block until the asynchronous operation has completed.
      do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

      // Determine whether a connection was successfully established.
      if (ec_ || ! socket_.is_open()) {
	if (ec_.value() == boost::system::errc::operation_canceled) {
	  throw Exception("Sick TiM55X Ethernet: connection timed out");
	} else {
	  throw Exception("Connection failed: %s", ec_.message().c_str());
	}
      }
      deadline_.expires_at(boost::posix_time::pos_infin);
    }
  } catch (boost::system::system_error &e) {
    throw Exception("Connection failed: %s", e.what());
  }
}


void
SickTiM55xEthernetAcquisitionThread::close_device()
{
  boost::system::error_code err;
  if (socket_.is_open()) {
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, err);
    socket_.close();
  }
}


void
SickTiM55xEthernetAcquisitionThread::flush_device()
{
  if (socket_.is_open()) {
    try {
      soft_deadline_.expires_from_now(boost::posix_time::milliseconds(RECEIVE_TIMEOUT));
      do {
	ec_ = boost::asio::error::would_block;
	bytes_read_ = 0;

	boost::asio::async_read_until(socket_, input_buffer_, '\03',
#if BOOST_VERSION >= 104800
				      (boost::lambda::var(ec_) = boost::lambda::_1,
				       boost::lambda::var(bytes_read_) = boost::lambda::_2));
#else
				      boost::bind(
				        &SickTiM55xEthernetAcquisitionThread::handle_read,
				        this,
				        boost::asio::placeholders::error,
				        boost::asio::placeholders::bytes_transferred
				      ));
#endif

	do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

      } while (bytes_read_ > 0);
      soft_deadline_.expires_from_now(boost::posix_time::pos_infin);
    } catch (boost::system::system_error &e) {
      // ignore, just assume done, if there really is an error we'll
      // catch it later on
    }
  }
}

void
SickTiM55xEthernetAcquisitionThread::send_with_reply(const char *request,
						     std::string *reply)
{
  MutexLocker lock(socket_mutex_);

  int request_length = strlen(request);

  try {
    boost::asio::write(socket_, boost::asio::buffer(request, request_length));

    deadline_.expires_from_now(boost::posix_time::milliseconds(RECEIVE_TIMEOUT));

    ec_ = boost::asio::error::would_block;
    bytes_read_ = 0;
    boost::asio::async_read_until(socket_, input_buffer_, '\03',
#if BOOST_VERSION >= 104800
				  (boost::lambda::var(ec_) = boost::lambda::_1,
				   boost::lambda::var(bytes_read_) = boost::lambda::_2));
#else
				  boost::bind(
				    &SickTiM55xEthernetAcquisitionThread::handle_read,
				    this,
				    boost::asio::placeholders::error,
				    boost::asio::placeholders::bytes_transferred
				  ));
#endif

    do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

    if (ec_) {
      if (ec_.value() == boost::system::errc::operation_canceled) {
	throw Exception("Timeout waiting for message reply");
      } else {
	throw Exception("Failed to read reply: %s", ec_.message().c_str());
      }
    }

    deadline_.expires_at(boost::posix_time::pos_infin);

    if (reply) {
      char recv_buf[bytes_read_];
      std::istream in_stream(&input_buffer_);
      in_stream.read(recv_buf, bytes_read_);
      *reply = std::string(recv_buf, bytes_read_);
    } else {
      input_buffer_.consume(bytes_read_);
    }
  } catch (boost::system::system_error &e) {
    throw Exception("Sick TiM55x/Ethernet failed I/O: %s", e.what());
  }
}


/** Check whether the deadline has passed.
 * We compare the deadline against
 * the current time since a new asynchronous operation may have moved the
 * deadline before this actor had a chance to run.
 */
void
SickTiM55xEthernetAcquisitionThread::check_deadline()
{
  if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
    socket_.close();
    deadline_.expires_at(boost::posix_time::pos_infin);
  }

#if BOOST_VERSION >= 104800
  deadline_.async_wait(boost::lambda::bind(&SickTiM55xEthernetAcquisitionThread::check_deadline, this));
#else
  deadline_.async_wait(boost::bind(&SickTiM55xEthernetAcquisitionThread::check_deadline, this));
#endif
}

/** Check whether the soft timeout deadline has passed.
 * We compare the deadline against the current time since a new
 * asynchronous operation may have moved the deadline before this
 * actor had a chance to run.
 */
void
SickTiM55xEthernetAcquisitionThread::check_soft_timeout()
{
  if (soft_deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
    socket_.cancel();
    soft_deadline_.expires_at(boost::posix_time::pos_infin);
  }

#if BOOST_VERSION >= 104800
  soft_deadline_.async_wait(boost::lambda::bind(&SickTiM55xEthernetAcquisitionThread::check_soft_timeout, this));
#else
  soft_deadline_.async_wait(boost::bind(&SickTiM55xEthernetAcquisitionThread::check_soft_timeout, this));
#endif
}
