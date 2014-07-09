
/***************************************************************************
 *  imu_cruizcore_xg1010.cpp - Retrieve IMU data from CruizCore XG1010
 *
 *  Created: Sun Jun 22 21:44:17 2014
 *  Copyright  2008-2014  Tim Niemueller [www.niemueller.de]
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

#include "imu_cruizcore_xg1010.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

#include <tf/types.h>
#include <utils/math/angle.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#include <boost/bind.hpp>

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>

using namespace fawkes;

#define RECONNECT_INTERVAL 2000

/** @class CruizCoreXG1010AcquisitionThread "imu_cruizcore_xg1010.h"
 * IMU acquisition thread for CruizCore XG1010 gyros.
 * @author Tim Niemueller
 */


/// @cond OLD_BOOST_COMPAT

#if BOOST_VERSION < 104800

// The default maximum number of bytes to transfer in a single operation.
enum { default_max_transfer_size = 65536 };

class transfer_exactly_t
{
 public:
  typedef std::size_t result_type;

  explicit transfer_exactly_t(std::size_t size)
    : size_(size) {}

  template <typename Error>
  std::size_t operator()(const Error& err, std::size_t bytes_transferred)
  {
    return (!!err || bytes_transferred >= size_) ? 0 :
      (size_ - bytes_transferred < default_max_transfer_size
        ? size_ - bytes_transferred : std::size_t(default_max_transfer_size));
  }

 private:
  std::size_t size_;
};

inline transfer_exactly_t transfer_exactly(std::size_t size)
{
  return transfer_exactly_t(size);
}
#endif

/// @endcond


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 * @param continuous true to run continuous, false otherwise
 */
CruizCoreXG1010AcquisitionThread::CruizCoreXG1010AcquisitionThread(std::string &cfg_name,
								   std::string &cfg_prefix,
								   bool continuous)
  : IMUAcquisitionThread(cfg_name.c_str(), continuous, cfg_name, cfg_prefix),
    serial_(io_service_), io_service_work_(io_service_), deadline_(io_service_)
{
  set_name("CruizCoreXG1010(%s)", cfg_name.c_str());
}

void
CruizCoreXG1010AcquisitionThread::init()
{
  deadline_.expires_at(boost::posix_time::pos_infin);
  check_deadline();

  cfg_serial_ = config->get_string((cfg_prefix_ + "device").c_str());
  cfg_baud_rate_ = config->get_uint((cfg_prefix_ + "baud_rate").c_str());
  cfg_freq_ = config->get_uint((cfg_prefix_ + "data_frequency").c_str());

  if (cfg_freq_ != 25 && cfg_freq_ != 50 && cfg_freq_ != 100) {
    throw Exception("Invalid data frequency, must be 25, 50, or 100");
  }
  if (cfg_baud_rate_ != 115200 && cfg_baud_rate_ != 57600 &&
      cfg_baud_rate_ != 38400 && cfg_baud_rate_ != 28800 &&
      cfg_baud_rate_ != 19200 && cfg_baud_rate_ != 9600 && cfg_baud_rate_ != 4800)
  {
    throw Exception("Invalid baud rate");
  }

  if ( (cfg_freq_ > 25 && cfg_baud_rate_ < 9600) ||
       (cfg_freq_ > 50 && cfg_baud_rate_ < 19200) )
  {
    throw Exception("Baud rate too low for frequency");
  }

  // wait up to two expected packets
  receive_timeout_ = (1000 / cfg_freq_) * 2;
  //logger->log_debug(name(), "Receive timeout: %u ms", receive_timeout_);

  // No acceleration data available, set to -1
  linear_acceleration_[0] = -1.;

  // from XG1010 data sheet
  angular_velocity_covariance_[8] = deg2rad(0.1);

  open_device();

  if (cfg_continuous_)  IMUAcquisitionThread::init();

#ifdef USE_TIMETRACKER
  tt_ = new TimeTracker();
  tt_loopcount_ = 0;
  ttc_full_loop_       = tt_->add_class("Full Loop");
  ttc_read_            = tt_->add_class("Read");
  ttc_catch_up_        = tt_->add_class("Catch up");
  ttc_parse_           = tt_->add_class("Parse");
#endif
}


void
CruizCoreXG1010AcquisitionThread::finalize()
{
  close_device();
  if (cfg_continuous_)  IMUAcquisitionThread::finalize();
#ifdef USE_TIMETRACKER
  delete tt_;
#endif
}


void
CruizCoreXG1010AcquisitionThread::loop()
{
  TIMETRACK_START(ttc_full_loop_);

  if (serial_.is_open()) {

    // reset data for the case that we timeout or fail
    angular_velocity_[2] = 0;
    orientation_[0] = orientation_[1] = orientation_[2] = orientation_[3] = 0.;

    try {
      TIMETRACK_START(ttc_read_);
      deadline_.expires_from_now(boost::posix_time::milliseconds(receive_timeout_));

      ec_ = boost::asio::error::would_block;
      bytes_read_ = 0;

      size_t to_read = CRUIZCORE_XG1010_PACKET_SIZE;
      // if there is partial data in the buffer we are running behind
      // read off old packet, catch up later will trigger if we had more data
      if (input_buffer_.size() > 0) {
	const size_t bsize = input_buffer_.size();
	size_t full_frames = bsize / CRUIZCORE_XG1010_PACKET_SIZE;
	size_t remaining =
	  CRUIZCORE_XG1010_PACKET_SIZE - (bsize - full_frames * CRUIZCORE_XG1010_PACKET_SIZE);
	to_read = remaining;
      }

      //logger->log_debug(name(), "Read %zu bytes", to_read);
      boost::asio::async_read(serial_, input_buffer_,
#if BOOST_VERSION >= 104800
			      boost::asio::transfer_exactly(to_read),
			      (boost::lambda::var(ec_) = boost::lambda::_1,
			       boost::lambda::var(bytes_read_) = boost::lambda::_2));
#else
			      transfer_exactly(to_read),
			      boost::bind(
				&CruizCoreXG1010AcquisitionThread::handle_read,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			      ));
#endif

      do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

      //logger->log_debug(name(), "Done");

      TIMETRACK_END(ttc_read_);

      data_mutex_->lock();
      timestamp_->stamp();
      data_mutex_->unlock();

      if (ec_) {
	if (ec_.value() == boost::system::errc::operation_canceled) {
	  logger->log_error(name(), "Data timeout, will try to reconnect");
	} else {
	  logger->log_warn(name(), "Data read error: %s\n", ec_.message().c_str());
	}
	data_mutex_->lock();
	new_data_ = true;
	data_mutex_->unlock();
	close_device();
      } else {
	TIMETRACK_START(ttc_catch_up_);
	bytes_read_ = 0;
	bool catch_up = false;
	size_t read_size = 0;
	do {
	  ec_ = boost::asio::error::would_block;

	  size_t to_read = CRUIZCORE_XG1010_PACKET_SIZE;
	  if (catch_up) {
	    deadline_.expires_from_now(boost::posix_time::milliseconds(receive_timeout_));
	    size_t full_frames = read_size / CRUIZCORE_XG1010_PACKET_SIZE;
	    size_t remaining = CRUIZCORE_XG1010_PACKET_SIZE - (read_size - full_frames * CRUIZCORE_XG1010_PACKET_SIZE);
	    to_read = remaining;
	  } else {
	    deadline_.expires_from_now(boost::posix_time::microseconds(10));
	  }
	  catch_up = false;

	  bytes_read_ = 0;

	  //logger->log_debug(name(), "Catch up read %zu bytes", to_read);
	  boost::asio::async_read(serial_, input_buffer_,
#if BOOST_VERSION >= 104800
	    boost::asio::transfer_exactly(to_read),
	    (boost::lambda::var(ec_) = boost::lambda::_1,
	     boost::lambda::var(bytes_read_) = boost::lambda::_2));
#else
	    transfer_exactly(to_read),
	    boost::bind(
		        &CruizCoreXG1010AcquisitionThread::handle_read,
		        this,
		        boost::asio::placeholders::error,
		        boost::asio::placeholders::bytes_transferred
		        ));
#endif

          do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

	  if (bytes_read_ > 0) {
	    read_size += bytes_read_;
	    catch_up = (read_size % CRUIZCORE_XG1010_PACKET_SIZE != 0);
	    ec_ = boost::system::error_code();
	  }
        } while (bytes_read_ > 0);
      }

      TIMETRACK_END(ttc_catch_up_);

      if (ec_ && ec_.value() != boost::system::errc::operation_canceled) {
	logger->log_warn(name(), "Data read error: %s\n", ec_.message().c_str());

	data_mutex_->lock();
	new_data_ = true;
	data_mutex_->unlock();
	close_device();
      } else {
	if (input_buffer_.size() >= CRUIZCORE_XG1010_PACKET_SIZE) {
	  TIMETRACK_START(ttc_parse_);
	  if (input_buffer_.size() > CRUIZCORE_XG1010_PACKET_SIZE) {
	    input_buffer_.consume(input_buffer_.size() - CRUIZCORE_XG1010_PACKET_SIZE);
	  }
	  std::istream in_stream(&input_buffer_);
	  in_stream.read((char *)in_packet_, CRUIZCORE_XG1010_PACKET_SIZE);

	  /*
	  printf("Packet (%zu): ", bytes_read_);
	  for (size_t i = 0; i < bytes_read_; ++i) {
	    printf("%x ", in_packet_[i] & 0xff);
	  }
	  printf("\n");
	  */

	  try {
	    parse_packet();
	  } catch (Exception &e) {
	    logger->log_warn(name(), e);
	    try {
	      resync();
	      logger->log_info(name(), "Successfully resynced");
	    } catch (Exception &e) {
	      logger->log_warn(name(), "Resync failed, trying to re-open");
	      close_device();
	    }
	  }
	  TIMETRACK_END(ttc_parse_);
	} else {
	  logger->log_warn(name(), "*** INVALID number of bytes in buffer: %zu\n", input_buffer_.size());
	}
      }
    } catch (boost::system::system_error &e) {
      if (e.code() == boost::asio::error::eof) {
	close_device();
	logger->log_warn(name(),
			 "CruizCoreXG1010 connection lost, trying to re-open");
      } else {
	logger->log_warn(name(), "CruizCore failed read: %s", e.what());
      }
    }
  } else {
    try {
      open_device();
      logger->log_warn(name(), "Reconnected to device");
    } catch (Exception &e) {
      // ignore, keep trying
      usleep(RECONNECT_INTERVAL * 1000);
    }
  }

  if (cfg_continuous_)  IMUAcquisitionThread::loop();

  yield();
  TIMETRACK_END(ttc_full_loop_);
#ifdef USE_TIMETRACKER
  if (++tt_loopcount_ >= 50) {
    tt_loopcount_ = 0;
    tt_->print_to_stdout();
  }
#endif
}


void
CruizCoreXG1010AcquisitionThread::open_device()
{
  try {
    input_buffer_.consume(input_buffer_.size());

    serial_.open(cfg_serial_);
    // according to CruizCore R1050K (sensor in XG1010) technical manual
    serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_.set_option(boost::asio::serial_port::baud_rate(cfg_baud_rate_));
    
    send_init_packet(/* enable transfer */ true);

    resync();
  } catch (boost::system::system_error &e) {
    throw Exception("CruizCore-XG1010 failed I/O: %s", e.what());
  }
}


void
CruizCoreXG1010AcquisitionThread::send_init_packet(bool enable_transfer)
{
  /* Format according to Technical Manual of R1050K for XG1010
     Field		Command		Separator	Example
     INIT		$MIA		COMMA (,)	$MIA,
     FORMAT		F, I or A	COMMA (,)	I,
     BAUD RATE	B,BAUDRATE	COMMA (,)	B,115200,
     OUTPUT RATE	R		COMMA (,)	R,100,
     TYPE		D or R		COMMA (,)	D,
     OUTPUT		Y or N		COMMA (,)	Y,
     FLASH		Y or N		COMMA (,)	Y,
     CHECKSUM		SUM of COMMAND	ASTERISK(*)	*C4
  */
  char *cmd_packet;
  if (asprintf(&cmd_packet, "$MIA,I,B,%u,R,%u,D,%s,N*  ", cfg_baud_rate_, cfg_freq_,
	       enable_transfer ? "Y" : "N") == -1)
  {
    throw Exception("Failed to create command packet");
  }

  size_t cmd_packet_len = strlen(cmd_packet);

  // calculate checksum
  unsigned int checksum = 0;
  for(size_t i = 1; i < cmd_packet_len - 3; ++i )  checksum += cmd_packet[i];
  checksum &= 0xFF;

  char checksum_str[3];
  snprintf(checksum_str, 3, "%X", checksum);
  cmd_packet[cmd_packet_len - 2] = checksum_str[0];
  cmd_packet[cmd_packet_len - 1] = checksum_str[1];

  std::string cmd_packet_s(cmd_packet, cmd_packet_len);
  free(cmd_packet);

  logger->log_debug(name(), "Sending init packet: %s", cmd_packet_s.c_str());

  boost::asio::write(serial_, boost::asio::buffer(cmd_packet_s.c_str(), cmd_packet_len));

}

void
CruizCoreXG1010AcquisitionThread::resync()
{
  /*
  // the idea here would be to:
  // 1. stop data transfer
  // 2. read all remaining data, i.e. flush serial line
  // 3. start data transfer again
  // 4. synchronize with packet stream
  // the only problem being that once the transfer has been stopped it
  // cannot be enabled again, this seems to be a bug in the CruizCore XG1010.

  // stop transfer and sniff off any data still arriving
  send_init_packet(/ enable transfer / false);
  try {
    do {
      ec_ = boost::asio::error::would_block;
      deadline_.expires_from_now(boost::posix_time::milliseconds(receive_timeout_));
      bytes_read_ = 0;

      boost::asio::async_read(serial_, input_buffer_,
#if BOOST_VERSION >= 104800
			      (boost::lambda::var(ec_) = boost::lambda::_1,
			       boost::lambda::var(bytes_read_) = boost::lambda::_2));
#else
       			      boost::bind(
				&CruizCoreXG1010AcquisitionThread::handle_read,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			      ));
#endif

      do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

      input_buffer_.consume(input_buffer_.size());

      logger->log_warn(name(), "EC: %s\n", ec_.message().c_str());
    } while (!ec_ && bytes_read_ > 0);
  } catch (boost::system::system_error &e) {
    // ignore, just assume done, if there really is an error we'll                   
    // catch it later on                                                             
  }

  send_init_packet(/ enable transfer / true);
  */

#if BOOST_VERSION >= 104700
  tcflush(serial_.lowest_layer().native_handle(), TCIOFLUSH);
#else
  tcflush(serial_.lowest_layer().native(), TCIOFLUSH);
#endif

  // Try 10 times
  const int NUM_TRIES = 10;
  for (int t = 1; t <= NUM_TRIES; ++t) {
    try {
      ec_ = boost::asio::error::would_block;
      bytes_read_ = 0;

      deadline_.expires_from_now(boost::posix_time::milliseconds(receive_timeout_ * 10));
      boost::asio::async_read_until(serial_, input_buffer_, std::string("\xff\xff"),
#if BOOST_VERSION >= 104800
				    (boost::lambda::var(ec_) = boost::lambda::_1,
				     boost::lambda::var(bytes_read_) = boost::lambda::_2));
#else
       				    boost::bind(
				      &CruizCoreXG1010AcquisitionThread::handle_read,
				      this,
				      boost::asio::placeholders::error,
				      boost::asio::placeholders::bytes_transferred
				    ));
#endif
    
      do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

      if (ec_) {
        if (ec_.value() == boost::system::errc::operation_canceled) {
          throw Exception("Timeout (1) on initial synchronization");
        } else {
          throw Exception("Error (1) on initial synchronization: %s", ec_.message().c_str());
        }
      }

      input_buffer_.consume(bytes_read_ - 2);

      deadline_.expires_from_now(boost::posix_time::milliseconds(receive_timeout_));
      ec_ = boost::asio::error::would_block;
      bytes_read_ = 0;
      boost::asio::async_read(serial_, input_buffer_,
#if BOOST_VERSION >= 104800
			      boost::asio::transfer_exactly(CRUIZCORE_XG1010_PACKET_SIZE - 2),
			      (boost::lambda::var(ec_) = boost::lambda::_1,
			       boost::lambda::var(bytes_read_) = boost::lambda::_2));
#else
			      transfer_exactly(CRUIZCORE_XG1010_PACKET_SIZE - 2),
			      boost::bind(
			        &CruizCoreXG1010AcquisitionThread::handle_read,
			        this,
			        boost::asio::placeholders::error,
			        boost::asio::placeholders::bytes_transferred
			      ));
#endif

      do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

      if (ec_) {
        if (ec_.value() == boost::system::errc::operation_canceled) {
          throw Exception("Timeout (2) on initial synchronization");
        } else {
          throw Exception("Error (2) on initial synchronization: %s", ec_.message().c_str());
        }
      }

      std::istream in_stream(&input_buffer_);
      in_stream.read((char *)in_packet_, CRUIZCORE_XG1010_PACKET_SIZE);

      parse_packet();
    } catch (Exception &e) {
      if (t == NUM_TRIES) {
	e.append("Resync failed after %d tries", NUM_TRIES);
	throw;
      }
      //else {
	//logger->log_warn(name(), "Resync retry %i failed, retrying %i more time%s",
	//		 t, NUM_TRIES - t, (NUM_TRIES - t == 1) ? "" : "s");
      //}
    }
  }
  deadline_.expires_at(boost::posix_time::pos_infin);
}


void
CruizCoreXG1010AcquisitionThread::close_device()
{
  serial_.close();
}

void
CruizCoreXG1010AcquisitionThread::parse_packet()
{
  /*
  printf("Packet: ");
  for (size_t i = 0; i < CRUIZCORE_XG1010_PACKET_SIZE; ++i) {
    printf("%x ", in_packet_[i] & 0xff);
  }
  printf("\n");
  */

  if( in_packet_[0] != 0xFF || in_packet_[1] != 0xFF ) {
    throw Exception("Failed to parse packet, invalid header");
  }

  short int rate  = ( in_packet_[2] & 0xFF) | ((in_packet_[3] << 8) & 0xFF00);
  short int angle = ( in_packet_[4] & 0xFF) | ((in_packet_[5] << 8) & 0XFF00);

  int checksum = 0xffff + rate + angle;
  if(((unsigned char)(checksum & 0xFF) != in_packet_[6]) ||
     ((unsigned char)((checksum>>8) & 0xFF) != in_packet_[7]))
  {
    /*
    printf("Packet: ");
    for (size_t i = 0; i < CRUIZCORE_XG1010_PACKET_SIZE; ++i) {
      printf("%x ", in_packet_[i] & 0xff);
    }
    printf("\n");
    */
    throw Exception("Failed to parse packet, checksum mismatch");
  }

  data_mutex_->lock();
  new_data_ = true;

  angular_velocity_[2] = -deg2rad(rate / 100.f);

  tf::Quaternion q = tf::create_quaternion_from_yaw(-deg2rad(angle / 100.f));
  orientation_[0] = q.x();
  orientation_[1] = q.y();
  orientation_[2] = q.z();
  orientation_[3] = q.w();

  data_mutex_->unlock();
}


/** Check whether the deadline has passed.
  * We compare the deadline against
  * the current time since a new asynchronous operation may have moved the
  * deadline before this actor had a chance to run.
  */
void
CruizCoreXG1010AcquisitionThread::check_deadline()
{
  if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
    serial_.cancel();
    deadline_.expires_at(boost::posix_time::pos_infin);
  }

#if BOOST_VERSION >= 104800
  deadline_.async_wait(boost::lambda::bind(&CruizCoreXG1010AcquisitionThread::check_deadline, this));
#else
  deadline_.async_wait(boost::bind(&CruizCoreXG1010AcquisitionThread::check_deadline, this));
#endif
}
