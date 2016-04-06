
/***************************************************************************
 *  com_thread.cpp - Robotino com thread
 *
 *  Created: Thu Sep 11 13:18:00 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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

#include "direct_com_thread.h"
#include "direct_com_message.h"
#include <baseapp/run.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>
#include <utils/time/wait.h>
#include <tf/types.h>

#include <interfaces/BatteryInterface.h>
#include <interfaces/RobotinoSensorInterface.h>
#include <interfaces/IMUInterface.h>

#include <unistd.h>

#include <libudev.h>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

using namespace fawkes;

/** @class DirectRobotinoComThread "openrobotino_com_thread.h"
 * Thread to communicate with Robotino via OpenRobotino API (v1 or v2).
 * @author Tim Niemueller
 */

/** Constructor. */
DirectRobotinoComThread::DirectRobotinoComThread()
	: RobotinoComThread("DirectRobotinoComThread"),
	  serial_(io_service_), io_service_work_(io_service_), deadline_(io_service_)
{
}


/** Destructor. */
DirectRobotinoComThread::~DirectRobotinoComThread()
{
}


void
DirectRobotinoComThread::init()
{
	cfg_hostname_ = config->get_string("/hardware/robotino/hostname");
	cfg_enable_gyro_ = config->get_bool("/hardware/robotino/gyro/enable");
	cfg_sensor_update_cycle_time_ =
		config->get_uint("/hardware/robotino/sensor_update_cycle_time");
	cfg_gripper_enabled_ = config->get_bool("/hardware/robotino/gripper/enable_gripper");

	last_seqnum_ = 0;
	time_wait_   = new TimeWait(clock, cfg_sensor_update_cycle_time_ * 1000);

	// -------------------------------------------------------------------------- //

	try {
		cfg_device_ = config->get_string(("/hardware/robotino/direct/device"));
	} catch (Exception &e) {
#ifdef HAVE_LIBUDEV
		cfg_device_ = find_device_udev();
#else
		throw Exception("No udev support, must configure serial device");
#endif
	}

	deadline_.expires_at(boost::posix_time::pos_infin);
	check_deadline();

	open_device();
}


void
DirectRobotinoComThread::finalize()
{
	delete time_wait_;
}

void
DirectRobotinoComThread::once()
{
	//reset_odometry();
}

void
DirectRobotinoComThread::loop()
{
	time_wait_->mark_start();


	time_wait_->wait();
}


void
DirectRobotinoComThread::reset_odometry()
{
}


bool
DirectRobotinoComThread::is_connected()
{
	return false;
}


void
DirectRobotinoComThread::get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t)
{
	MutexLocker lock(data_mutex_);
}


void
DirectRobotinoComThread::get_odometry(double &x, double &y, double &phi)
{
	MutexLocker lock(data_mutex_);
}

bool
DirectRobotinoComThread::is_gripper_open()
{
	MutexLocker lock(data_mutex_);
	return false;
}

void
DirectRobotinoComThread::set_speed_points(float s1, float s2, float s3)
{
}

void
DirectRobotinoComThread::set_gripper(bool opened)
{
}

void
DirectRobotinoComThread::set_bumper_estop_enabled(bool enabled)
{
}


std::string
DirectRobotinoComThread::find_device_udev()
{
	std::string cfg_device = "";

	// try to find device using udev
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev, *usb_device;
	udev = udev_new();
	if (! udev) {
		throw Exception("RobotinoDirect: Failed to initialize udev for "
		                "device detection");
	}

	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "tty");
	udev_enumerate_scan_devices(enumerate);

	devices = udev_enumerate_get_list_entry(enumerate);
	udev_list_entry_foreach(dev_list_entry, devices) {
		const char *path;

		path = udev_list_entry_get_name(dev_list_entry);
		if (! path)  continue;
		
		dev = udev_device_new_from_syspath(udev, path);
		usb_device = udev_device_get_parent_with_subsystem_devtype(dev, "usb",
		                                                           "usb_device");
		if (! dev || ! usb_device) continue;

		std::string vendor_id = udev_device_get_property_value(dev, "ID_VENDOR_ID");
		std::string model_id  = udev_device_get_property_value(dev, "ID_MODEL_ID");

		if (vendor_id == "1e29" && model_id == "040d") {
			// found Robotino 3 device
			cfg_device = udev_device_get_property_value(dev, "DEVNAME");

			/*
			  struct udev_list_entry * props =
			  udev_device_get_properties_list_entry(dev);
			  udev_list_entry *p;
			  udev_list_entry_foreach(p, props)
			  {
			  printf("  %s = %s\n", udev_list_entry_get_name(p), udev_list_entry_get_value(p));
			  }
			*/

			std::string vendor = udev_device_get_property_value(dev, "ID_VENDOR_FROM_DATABASE");
			std::string model = "unknown";
			const char *model_from_db = udev_device_get_property_value(dev, "ID_MODEL_FROM_DATABASE");
			if (model_from_db) {
				model = model_from_db;
			} else {
				model = udev_device_get_property_value(dev, "ID_MODEL");
			}
			logger->log_debug(name(), "Found %s %s at device %s",
			                  vendor.c_str(), model.c_str(), cfg_device.c_str());
			break;
		}
	}

	udev_enumerate_unref(enumerate);
	udev_unref(udev);

	if (cfg_device == "") {
		throw Exception("No robotino device was found");
	}

	return cfg_device;
}

void
DirectRobotinoComThread::open_device()
{
	try {
		input_buffer_.consume(input_buffer_.size());

		serial_.open(cfg_device_);
		//serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::none));
		serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
		serial_.set_option(boost::asio::serial_port::baud_rate(115200));
    
		//send_init_packet(/* enable transfer */ true);

		//resync();
	} catch (boost::system::system_error &e) {
		throw Exception("RobotinoDirect failed I/O: %s", e.what());
	}

	{
		DirectRobotinoComMessage req;
		req.add_command(DirectRobotinoComMessage::CMDID_GET_HW_VERSION);
		req.add_command(DirectRobotinoComMessage::CMDID_GET_SW_VERSION);
		DirectRobotinoComMessage::pointer m = send_and_recv(req);

		//logger->log_info(name(), "Escaped:\n%s", m->to_string(true).c_str());
		//logger->log_info(name(), "Un-Escaped:\n%s", m->to_string(false).c_str());
		std::string hw_version, sw_version;
		DirectRobotinoComMessage::command_id_t msgid;
		while ((msgid = m->next_command()) != DirectRobotinoComMessage::CMDID_NONE) {
			if (msgid == DirectRobotinoComMessage::CMDID_HW_VERSION) {
				hw_version = m->get_string();
			} else if (msgid == DirectRobotinoComMessage::CMDID_SW_VERSION) {
				sw_version = m->get_string();
				
			} else if (msgid == DirectRobotinoComMessage::CMDID_CHARGER_ERROR) {
				uint8_t id = m->get_uint8();
				uint32_t mtime = m->get_uint32();
				std::string error = m->get_string();
				logger->log_warn(name(), "Charger error (ID %u, Time: %u): %s",
				                 id, mtime, error.c_str());
			}
		}
		if (hw_version.empty() || sw_version.empty()) {
			close_device();
			throw Exception("RobotinoDirect: no reply to version inquiry from robot");
		}
		logger->log_debug(name(), "Connected, HW Version: %s  SW Version: %s",
		                  hw_version.c_str(), sw_version.c_str());
	}
}


void
DirectRobotinoComThread::close_device()
{
	serial_.cancel();
	serial_.close();
}

void
DirectRobotinoComThread::send_message(DirectRobotinoComMessage &msg)
{
	boost::asio::write(serial_, boost::asio::const_buffers_1(msg.buffer()));
}

/*
  std::shared_ptr<DirectRobotinoComMessage>
  DirectRobotinoComMessage::send_and_recv(std::shared_ptr<DirectRobotinoComMessage> msg)
  {
	
  }
*/

std::shared_ptr<DirectRobotinoComMessage>
DirectRobotinoComThread::send_and_recv(DirectRobotinoComMessage &msg)
{
	send_message(msg);
	read_packet();

	std::shared_ptr<DirectRobotinoComMessage> m =
		std::make_shared<DirectRobotinoComMessage>(boost::asio::buffer_cast<const unsigned char*>(input_buffer_.data()),
		                                           input_buffer_.size());

	input_buffer_.consume(input_buffer_.size());
	return m;
}

/// @cond INTERNAL
/** Matcher to count unescaped number of bytes. */
class match_unescaped_length
{
public:
	explicit match_unescaped_length(unsigned short length) : length_(length), l_(0) {}

	template <typename Iterator>
	std::pair<Iterator, bool> operator()(
	                                     Iterator begin, Iterator end) const
	{
		Iterator i = begin;
		while (i != end && l_ < length_) {
			if (*i++ != DirectRobotinoComMessage::MSG_DATA_ESCAPE) {
				l_ += 1;
			}
		}
		return std::make_pair(i, (l_ == length_));
	}

private:
	unsigned short length_;
	mutable unsigned short l_;
};

namespace boost {
	namespace asio {
		template <> struct is_match_condition<match_unescaped_length>
			: public boost::true_type {};
	} // namespace asio
}
/// @endcond

void
DirectRobotinoComThread::read_packet()
{
	boost::system::error_code ec = boost::asio::error::would_block;
	size_t bytes_read = 0;

	deadline_.expires_from_now(boost::posix_time::milliseconds(50));
	boost::asio::async_read_until(serial_, input_buffer_, DirectRobotinoComMessage::MSG_HEAD,
	                              (boost::lambda::var(ec) = boost::lambda::_1,
	                               boost::lambda::var(bytes_read) = boost::lambda::_2));
    
	do io_service_.run_one(); while (ec == boost::asio::error::would_block);

	if (ec) {
		if (ec.value() == boost::system::errc::operation_canceled) {
			throw Exception("Timeout (1) on initial synchronization");
		} else {
			throw Exception("Error (1) on initial synchronization: %s", ec.message().c_str());
		}
	}

	// Read all potential junk before the start header
	input_buffer_.consume(bytes_read - 1);

	// read packet length
	ec = boost::asio::error::would_block;
	bytes_read = 0;
	boost::asio::async_read_until(serial_, input_buffer_,
	                              match_unescaped_length(2),
	                              (boost::lambda::var(ec) = boost::lambda::_1,
	                               boost::lambda::var(bytes_read) = boost::lambda::_2));

	do io_service_.run_one(); while (ec == boost::asio::error::would_block);

	if (ec) {
		if (ec.value() == boost::system::errc::operation_canceled) {
			throw Exception("Timeout (2) on initial synchronization");
		} else {
			throw Exception("Error (2) on initial synchronization: %s", ec.message().c_str());
		}
	}

	const unsigned char *length_escaped =
		boost::asio::buffer_cast<const unsigned char*>(input_buffer_.data());

	unsigned char *length_unescaped = (unsigned char *)malloc(bytes_read);
	size_t length_unescaped_size __attribute__((unused)) =
		DirectRobotinoComMessage::unescape(length_unescaped, &length_escaped[1], bytes_read);

	// if (length_unescaped_size != 2) fail
	unsigned short length =
		DirectRobotinoComMessage::parse_uint16(length_unescaped);

	// read remaining packet
	ec = boost::asio::error::would_block;
	bytes_read = 0;
	boost::asio::async_read_until(serial_, input_buffer_,
	                              match_unescaped_length(length),
	                              (boost::lambda::var(ec) = boost::lambda::_1,
	                               boost::lambda::var(bytes_read) = boost::lambda::_2));

	do io_service_.run_one(); while (ec == boost::asio::error::would_block);

	if (ec) {
		if (ec.value() == boost::system::errc::operation_canceled) {
			throw Exception("Timeout (3) on initial synchronization");
		} else {
			throw Exception("Error (3) on initial synchronization: %s", ec.message().c_str());
		}
	}
}


/** Check whether the deadline has passed.
 * We compare the deadline against
 * the current time since a new asynchronous operation may have moved the
 * deadline before this actor had a chance to run.
 */
void
DirectRobotinoComThread::check_deadline()
{
	if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
		serial_.cancel();
		deadline_.expires_at(boost::posix_time::pos_infin);
	}

	deadline_.async_wait(boost::lambda::bind(&DirectRobotinoComThread::check_deadline, this));
}
