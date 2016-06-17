
/***************************************************************************
 *  direct_com_thread.cpp - Robotino com thread for direct communication
 *
 *  Created: Mon Apr 04 11:48:36 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
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
#include <boost/bind.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/filesystem.hpp>

using namespace fawkes;

/** @class DirectRobotinoComThread "openrobotino_com_thread.h"
 * Thread to communicate with Robotino via OpenRobotino API (v1 or v2).
 * @author Tim Niemueller
 */

/** Constructor. */
DirectRobotinoComThread::DirectRobotinoComThread()
	: RobotinoComThread("DirectRobotinoComThread"),
	  serial_(io_service_), io_service_work_(io_service_), deadline_(io_service_),
	  request_timer_(io_service_), nodata_timer_(io_service_), drive_timer_(io_service_)
{
	set_prepfin_conc_loop(true);
}


/** Destructor. */
DirectRobotinoComThread::~DirectRobotinoComThread()
{
}


void
DirectRobotinoComThread::init()
{
	cfg_enable_gyro_ = config->get_bool("/hardware/robotino/gyro/enable");
	cfg_sensor_update_cycle_time_ =
		config->get_uint("/hardware/robotino/cycle-time");
	cfg_gripper_enabled_ = config->get_bool("/hardware/robotino/gripper/enable_gripper");
	cfg_rpm_max_ = config->get_float("/hardware/robotino/motor/rpm-max");
	cfg_nodata_timeout_ = config->get_uint("/hardware/robotino/direct/no-data-timeout");
	cfg_drive_update_interval_ = config->get_uint("/hardware/robotino/direct/drive-update-interval");
	cfg_read_timeout_ = config->get_uint("/hardware/robotino/direct/read-timeout");
	cfg_log_checksum_errors_ = config->get_bool("/hardware/robotino/direct/checksums/log-errors");
	cfg_checksum_error_recover_ = config->get_uint("/hardware/robotino/direct/checksums/recover-bound");
	cfg_checksum_error_critical_ = config->get_uint("/hardware/robotino/direct/checksums/critical-bound");
	
	// -------------------------------------------------------------------------- //

	if (find_controld3()) {
		throw Exception("Found running controld3, stop using 'sudo initctl stop controld3'");
	}

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

	request_timer_.expires_from_now(boost::posix_time::milliseconds(-1));
	drive_timer_.expires_at(boost::posix_time::pos_infin);

	digital_outputs_ = 0;

	open_device(/* wait for replies */ true);
	open_tries_ = 0;
	checksum_errors_ = 0;

	{ // Disable all digital outputs initially
		DirectRobotinoComMessage req(DirectRobotinoComMessage::CMDID_SET_ALL_DIGITAL_OUTPUTS);
		req.add_uint8(digital_outputs_);
		send_message(req);
	}
}


bool
DirectRobotinoComThread::prepare_finalize_user()
{
	//logger->log_info(name(), "Prepare Finalize");
	request_timer_.cancel();
	nodata_timer_.cancel();
	drive_timer_.cancel();
	drive_timer_.expires_at(boost::posix_time::pos_infin);
	request_timer_.expires_at(boost::posix_time::pos_infin);
	nodata_timer_.expires_at(boost::posix_time::pos_infin);
	deadline_.expires_at(boost::posix_time::pos_infin);

	serial_.cancel();
	
	return true;
}

void
DirectRobotinoComThread::finalize()
{
	close_device();
}

void
DirectRobotinoComThread::once()
{
	reset_odometry();
	request_data();
	update_nodata_timer();
}

void
DirectRobotinoComThread::loop()
{
	if (finalize_prepared) {
		usleep(1000);
		return;
	}

	if (opened_) {
		try {
			DirectRobotinoComMessage::pointer m = read_packet();
			checksum_errors_ = 0;
			process_message(m);
			update_nodata_timer();
		} catch (DirectRobotinoComMessage::ChecksumError &ce) {
			input_buffer_.consume(input_buffer_.size());
			if (! finalize_prepared && opened_) {
				checksum_errors_ += 1;
				if (cfg_log_checksum_errors_) {
					logger->log_warn(name(), "%s [%u consecutive errors]",
					                 ce.what_no_backtrace(), checksum_errors_);
				}
				if (checksum_errors_ >= cfg_checksum_error_recover_) {
					logger->log_warn(name(), "Large number of consecutive checksum errors, trying recover");
					input_buffer_.consume(input_buffer_.size());
					try {
						DirectRobotinoComMessage req(DirectRobotinoComMessage::CMDID_GET_HW_VERSION);
						send_message(req);
						request_data();
					} catch (Exception &e) {}
				} else if (checksum_errors_ >= cfg_checksum_error_critical_) {
					logger->log_error(name(), "Critical number of consecutive checksum errors, re-opening");
					input_buffer_.consume(input_buffer_.size());
					close_device();
				}
			}
		} catch (Exception &e) {
			if (! finalize_prepared) {
				if (opened_) {
					logger->log_warn(name(), "Transmission error, sending ping");
					logger->log_warn(name(), e);
					input_buffer_.consume(input_buffer_.size());
					try {
						DirectRobotinoComMessage req(DirectRobotinoComMessage::CMDID_GET_HW_VERSION);
						send_message(req);
						request_data();
					} catch (Exception &e) {}
				} else {
					logger->log_warn(name(), "Transmission error, connection closed");
				}
			}
		}
	} else {
		try {
			logger->log_info(name(), "Re-opening device");
			open_device(/* wait for replies */ false);
			logger->log_info(name(), "Connection re-established after %u tries", open_tries_ + 1);
			open_tries_ = 0;
			request_data();
		} catch (Exception &e) {
			logger->log_warn(name(), "Re-open failed: %s", e.what_no_backtrace());
			open_tries_ += 1;
			if (open_tries_ >= (1000 / cfg_sensor_update_cycle_time_)) {
				logger->log_error(name(), "Connection problem to base persists");
				open_tries_ = 0;
			}
		}
	}
}


void
DirectRobotinoComThread::process_message(DirectRobotinoComMessage::pointer m)
{
	bool new_data = false;

	DirectRobotinoComMessage::command_id_t msgid;
	while ((msgid = m->next_command()) != DirectRobotinoComMessage::CMDID_NONE) {
		//logger->log_info(name(), "Command length: %u", m->command_length());

		if (msgid == DirectRobotinoComMessage::CMDID_ALL_MOTOR_READINGS) {
			// there are four motors, one of which might be a gripper, therefore skips

			for (int i = 0; i < 3; ++i)  data_.mot_velocity[i] = m->get_int16();
			m->skip_int16();

			for (int i = 0; i < 3; ++i)  data_.mot_position[i] = m->get_int32();
			m->skip_int32();

			for (int i = 0; i < 3; ++i)  data_.mot_current[i] = m->get_float();
			new_data = true;

		} else if (msgid == DirectRobotinoComMessage::CMDID_DISTANCE_SENSOR_READINGS) {
			for (int i = 0; i < 9; ++i)  data_.ir_voltages[i] = m->get_float();
			new_data = true;

		} else if (msgid == DirectRobotinoComMessage::CMDID_ALL_ANALOG_INPUTS) {
			for (int i = 0; i < 8; ++i)  data_.analog_in[i] = m->get_float();
			new_data = true;

		} else if (msgid == DirectRobotinoComMessage::CMDID_ALL_DIGITAL_INPUTS) {
			uint8_t value = m->get_uint8();
			for (int i = 0; i < 8; ++i)  data_.digital_in[i] = (value & (1 << i)) ? true : false;
			new_data = true;

		} else if (msgid == DirectRobotinoComMessage::CMDID_BUMPER) {
			data_.bumper = (m->get_uint8() != 0) ? true : false;
			new_data = true;

		} else if (msgid == DirectRobotinoComMessage::CMDID_ODOMETRY) {
			data_.odo_x   = m->get_float();
			data_.odo_y   = m->get_float();
			data_.odo_phi = m->get_float();
			new_data = true;

		} else if (msgid == DirectRobotinoComMessage::CMDID_POWER_SOURCE_READINGS) {
			float voltage = m->get_float();
			float current = m->get_float();

			data_.bat_voltage = voltage * 1000.; // V -> mV
			data_.bat_current = current * 1000.; // A -> mA

			// 22.0V is empty, 24.5V is full, this is just a guess
			float soc = (voltage - 22.0f) / 2.5f;
			soc = std::min(1.f, std::max(0.f, soc));
			data_.bat_absolute_soc = soc;

			new_data = true;

		} else if (msgid == DirectRobotinoComMessage::CMDID_CHARGER_ERROR) {
			uint8_t id = m->get_uint8();
			uint32_t mtime = m->get_uint32();
			std::string error = m->get_string();
			logger->log_warn(name(), "Charger error (ID %u, Time: %u): %s",
			                 id, mtime, error.c_str());
		}
	}

	if (new_data) {
		MutexLocker lock(data_mutex_);
		new_data_ = true;
		data_.seq += 1;
		data_.time.stamp();
	}
}


void
DirectRobotinoComThread::reset_odometry()
{
	try {
		DirectRobotinoComMessage m(DirectRobotinoComMessage::CMDID_SET_ODOMETRY);
		m.add_float(0.); // X (m)
		m.add_float(0.); // Y (m)
		m.add_float(0.); // rot (rad)
		send_message(m);
	} catch (Exception &e) {
		logger->log_error(name(), "Resetting odometry failed, exception follows");
		logger->log_error(name(), e);
	}
}


void
DirectRobotinoComThread::set_motor_accel_limits(float min_accel, float max_accel)
{
	try {
		DirectRobotinoComMessage req;
		for (int i = 0; i < 2; ++i) {
			req.add_command(DirectRobotinoComMessage::CMDID_SET_MOTOR_ACCEL_LIMITS);
			req.add_uint8(i);
			req.add_float(min_accel);
			req.add_float(max_accel);
		}
		send_message(req);
	} catch (Exception &e) {
		logger->log_error(name(), "Setting motor accel limits failed, exception follows");
		logger->log_error(name(), e);
	}
}

void
DirectRobotinoComThread::set_digital_output(unsigned int digital_out, bool enable)
{
	if (digital_out < 1 || digital_out > 8) {
		throw Exception("Invalid digital output, must be in range [1..8], got %u",
		                digital_out);
	}

	unsigned int digital_out_idx = digital_out - 1;

	if (enable) {
		digital_outputs_ |= (1 << digital_out_idx);
	} else {
		digital_outputs_ &= ~(1 << digital_out_idx);
	}
	
	try {
		DirectRobotinoComMessage req(DirectRobotinoComMessage::CMDID_SET_ALL_DIGITAL_OUTPUTS);
		req.add_uint8(digital_outputs_);
		send_message(req);
	} catch (Exception &e) {
		logger->log_error(name(), "Setting digital outputs failed, exception follows");
		logger->log_error(name(), e);
	}

	MutexLocker lock(data_mutex_);
	for (int i = 0; i < 8; ++i)  data_.digital_out[i] = (digital_outputs_ & (1 << i)) ? true : false;
	new_data_ = true;
}


bool
DirectRobotinoComThread::is_connected()
{
	return serial_.is_open();
}


void
DirectRobotinoComThread::get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t)
{
	MutexLocker lock(data_mutex_);
	a1 = data_.mot_velocity[0];
	a2 = data_.mot_velocity[1];
	a3 = data_.mot_velocity[2];

	seq = data_.seq;
	t   = data_.time;
}


void
DirectRobotinoComThread::get_odometry(double &x, double &y, double &phi)
{
	MutexLocker lock(data_mutex_);
	x   = data_.odo_x;
	y   = data_.odo_y;
	phi = data_.odo_phi;
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
	float bounded_s1 = std::max(-cfg_rpm_max_, std::min(cfg_rpm_max_, s1));
	float bounded_s2 = std::max(-cfg_rpm_max_, std::min(cfg_rpm_max_, s2));
	float bounded_s3 = std::max(-cfg_rpm_max_, std::min(cfg_rpm_max_, s3));
	
	try {
		DirectRobotinoComMessage m;
		m.add_command(DirectRobotinoComMessage::CMDID_SET_MOTOR_SPEED);
		m.add_uint8(0);
		m.add_uint16((uint16_t)roundf(bounded_s1));
		m.add_command(DirectRobotinoComMessage::CMDID_SET_MOTOR_SPEED);
		m.add_uint8(1);
		m.add_uint16((uint16_t)roundf(bounded_s2));
		m.add_command(DirectRobotinoComMessage::CMDID_SET_MOTOR_SPEED);
		m.add_uint8(2);
		m.add_uint16((uint16_t)roundf(bounded_s3));
		send_message(m);
	} catch (Exception &e) {
		logger->log_error(name(), "Setting speed points failed, exception follows");
		logger->log_error(name(), e);
	}
}

void
DirectRobotinoComThread::set_gripper(bool opened)
{
}

void
DirectRobotinoComThread::set_bumper_estop_enabled(bool enabled)
{
	try {
		DirectRobotinoComMessage m(DirectRobotinoComMessage::CMDID_SET_EMERGENCY_BUMPER);
		m.add_uint8(enabled ? 1 : 0);
		send_message(m);

		MutexLocker lock(data_mutex_);
		data_.bumper_estop_enabled = enabled;
	} catch (Exception &e) {
		logger->log_error(name(), "Setting bumper estop state failed, exception follows");
		logger->log_error(name(), e);
	}
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


bool
DirectRobotinoComThread::find_controld3()
{
	bool rv = false;
	boost::filesystem::path p("/proc");

	using namespace boost::filesystem;
 
  try {
	  if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p)) {

		  directory_iterator di;
		  for (di = directory_iterator(p); di != directory_iterator(); ++di) {
			  directory_entry &d = *di;
			  //for (directory_entry &d : directory_iterator(p))
			  std::string f = d.path().filename().native();
			  bool is_process = true;
			  for (std::string::size_type i = 0; i < f.length(); ++i) {
				  if (! isdigit(f[i])) {
					  is_process = false;
					  break;
				  }
			  }
			  if (is_process) {
				  path pproc(d.path());
				  pproc /= "stat";

				  FILE *f = fopen(pproc.c_str(), "r");
				  if (f) {
					  int pid;
					  char *procname;
					  if (fscanf(f, "%d (%m[a-z0-9])", &pid, &procname) == 2) {
						  
						  if (strcmp("controld3", procname) == 0) {
							  rv = true;
						  }
						  ::free(procname);
					  }
					  fclose(f);
				  }
			  }
		  }

	  } else {
		  logger->log_warn(name(), "Cannot open /proc, cannot determine if controld3 is running");
		  return false;
	  }

  } catch (const boost::filesystem::filesystem_error &ex) {
	  logger->log_warn(name(), "Failure to determine if controld3 is running: %s", ex.what());
	  return false;
  }

  return rv;
}

void
DirectRobotinoComThread::open_device(bool wait_replies)
{
	if (finalize_prepared)  return;

	try {
		input_buffer_.consume(input_buffer_.size());

		boost::mutex::scoped_lock lock(io_mutex_);

		serial_.open(cfg_device_);
		//serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::none));
		serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
		serial_.set_option(boost::asio::serial_port::baud_rate(115200));
		
		opened_ = true;
	} catch (boost::system::system_error &e) {
		throw Exception("RobotinoDirect failed I/O: %s", e.what());
	}

	{
		DirectRobotinoComMessage req;
		req.add_command(DirectRobotinoComMessage::CMDID_GET_HW_VERSION);
		req.add_command(DirectRobotinoComMessage::CMDID_GET_SW_VERSION);

		if (wait_replies) {
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
					//} else {
					//logger->log_debug(name(), "  - %u\n", msgid);
				}
			}
			if (hw_version.empty() || sw_version.empty()) {
				close_device();
				throw Exception("RobotinoDirect: no reply to version inquiry from robot");
			}
			logger->log_debug(name(), "Connected, HW Version: %s  SW Version: %s",
			                  hw_version.c_str(), sw_version.c_str());
		} else {
			try {
				send_message(req);
			} catch (Exception &e) {
				logger->log_error(name(), "Requesting version information failed, exception follows");
				logger->log_error(name(), e);
			}
		}
	}
}


void
DirectRobotinoComThread::close_device()
{
	serial_.cancel();
	serial_.close();
	opened_ = false;
	open_tries_ = 0;
}


void
DirectRobotinoComThread::flush_device()
{
	if (serial_.is_open()) {
		try {
			boost::system::error_code ec = boost::asio::error::would_block;
			size_t bytes_read = 0;
			do {
				ec = boost::asio::error::would_block;
				bytes_read = 0;

				deadline_.expires_from_now(boost::posix_time::milliseconds(cfg_read_timeout_));
				boost::asio::async_read(serial_, input_buffer_,
				                        boost::asio::transfer_at_least(1),
				                        (boost::lambda::var(ec) = boost::lambda::_1,
				                         boost::lambda::var(bytes_read) = boost::lambda::_2));

				do io_service_.run_one(); while (ec == boost::asio::error::would_block);

				if (bytes_read > 0) {
					logger->log_warn(name(), "Flushing %zu bytes\n", bytes_read);
				}

			} while (bytes_read > 0);
			deadline_.expires_from_now(boost::posix_time::pos_infin);
		} catch (boost::system::system_error &e) {
			// ignore, just assume done, if there really is an error we'll
			// catch it later on
		}
	}
}

void
DirectRobotinoComThread::send_message(DirectRobotinoComMessage &msg)
{
	boost::mutex::scoped_lock lock(io_mutex_);
	if (opened_) {
		//logger->log_warn(name(), "Sending");
		boost::system::error_code ec;
		boost::asio::write(serial_, boost::asio::const_buffers_1(msg.buffer()), ec);

		if (ec) {
			close_device();
			throw Exception("Error while writing message (%s), closing connection",
			                ec.message().c_str());
		}
	}
}

std::shared_ptr<DirectRobotinoComMessage>
DirectRobotinoComThread::send_and_recv(DirectRobotinoComMessage &msg)
{
	boost::mutex::scoped_lock lock(io_mutex_);
	if (opened_) {
		boost::system::error_code ec;
		boost::asio::write(serial_, boost::asio::const_buffers_1(msg.buffer()), ec);
		if (ec) {
			logger->log_error(name(), "Error while writing message (%s), closing connection",
			                  ec.message().c_str());

			close_device();
			throw Exception("RobotinoDirect: write failed (%s)", ec.message().c_str());
		}
		std::shared_ptr<DirectRobotinoComMessage> m = read_packet();
		return m;
	} else {
		throw Exception("RobotinoDirect: serial device not opened");
	}
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

DirectRobotinoComMessage::pointer
DirectRobotinoComThread::read_packet()
{
	boost::system::error_code ec = boost::asio::error::would_block;
	size_t bytes_read = 0;

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
	// if (bytes_read > 1) {
	// 	logger->log_warn(name(), "Read junk off line");
	// }
	input_buffer_.consume(bytes_read - 1);
	
	// start timeout for remaining packet
	deadline_.expires_from_now(boost::posix_time::milliseconds(cfg_read_timeout_));

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

	unsigned char length_unescaped[2];
	DirectRobotinoComMessage::unescape(length_unescaped, 2, &length_escaped[1], bytes_read);

	unsigned short length =
		DirectRobotinoComMessage::parse_uint16(length_unescaped);

	// read remaining packet
	ec = boost::asio::error::would_block;
	bytes_read = 0;
	boost::asio::async_read_until(serial_, input_buffer_,
	                              match_unescaped_length(length + 2), // +2: checksum
	                              (boost::lambda::var(ec) = boost::lambda::_1,
	                               boost::lambda::var(bytes_read) = boost::lambda::_2));

	do io_service_.run_one(); while (ec == boost::asio::error::would_block);

	if (ec) {
		if (ec.value() == boost::system::errc::operation_canceled) {
			throw Exception("Timeout (3) on initial synchronization (reading %u bytes, have %zu)",
			                length, input_buffer_.size());
		} else {
			throw Exception("Error (3) on initial synchronization: %s", ec.message().c_str());
		}
	}

	deadline_.expires_at(boost::posix_time::pos_infin);

	DirectRobotinoComMessage::pointer m = std::make_shared<DirectRobotinoComMessage>
		(boost::asio::buffer_cast<const unsigned char*> (input_buffer_.data()), input_buffer_.size());

	input_buffer_.consume(m->escaped_data_size());

	return m;
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


/** Request new data.
 */
void
DirectRobotinoComThread::request_data()
{
	if (finalize_prepared)  return;

	if (request_timer_.expires_from_now() < boost::posix_time::milliseconds(0)) {
		request_timer_.expires_from_now(boost::posix_time::milliseconds(cfg_sensor_update_cycle_time_));
		request_timer_.async_wait(boost::bind(&DirectRobotinoComThread::handle_request_data, this,
		                                      boost::asio::placeholders::error));
	}
}

void
DirectRobotinoComThread::handle_request_data(const boost::system::error_code &ec)
{
	if (! ec) {
		DirectRobotinoComMessage req;
		req.add_command(DirectRobotinoComMessage::CMDID_GET_ALL_MOTOR_READINGS);
		req.add_command(DirectRobotinoComMessage::CMDID_GET_DISTANCE_SENSOR_READINGS);
		req.add_command(DirectRobotinoComMessage::CMDID_GET_ALL_ANALOG_INPUTS);
		req.add_command(DirectRobotinoComMessage::CMDID_GET_ALL_DIGITAL_INPUTS);
		req.add_command(DirectRobotinoComMessage::CMDID_GET_BUMPER);
		req.add_command(DirectRobotinoComMessage::CMDID_GET_ODOMETRY);
		req.add_command(DirectRobotinoComMessage::CMDID_GET_GYRO_Z_ANGLE);
		// This command is documented in the wiki but does not exist in the
		// enum and is not handled in the firmware. Instead, the information
		// is sent with every reply, so we also get it here automatically.
		// This has been checked in Robotino3 firmware 1.1.1
		// req.add_command(DirectRobotinoComMessage::CMDID_GET_POWER_SOURCE_READINGS);

		//req.pack();
		//logger->log_debug(name(), "Req1:\n%s", req.to_string().c_str());

		try {
			send_message(req);
		} catch (Exception &e) {
			logger->log_warn(name(), "Sending message failed, excpeption follows");
			logger->log_warn(name(), e);
		}

	} else {		
		logger->log_warn(name(), "Request timer failed: %s", ec.message().c_str());
	}

	if (! finalize_prepared && opened_) {
		request_data();
	}
}

void
DirectRobotinoComThread::set_desired_vel(float vx, float vy, float omega)
{
	RobotinoComThread::set_desired_vel(vx, vy, omega);
	drive();
}


void
DirectRobotinoComThread::drive()
{
	if (finalize_prepared)  return;

	drive_timer_.expires_from_now(boost::posix_time::milliseconds(cfg_drive_update_interval_));
	drive_timer_.async_wait(boost::bind(&DirectRobotinoComThread::handle_drive, this,
	                                    boost::asio::placeholders::error));	
}


void
DirectRobotinoComThread::handle_drive(const boost::system::error_code &ec)
{
	if (! ec) {
		if (update_velocities())  drive();
	}
}


void
DirectRobotinoComThread::update_nodata_timer()
{
	nodata_timer_.cancel();
	nodata_timer_.expires_from_now(boost::posix_time::milliseconds(cfg_nodata_timeout_));
	nodata_timer_.async_wait(boost::bind(&DirectRobotinoComThread::handle_nodata, this,
	                                     boost::asio::placeholders::error));
}

void
DirectRobotinoComThread::handle_nodata(const boost::system::error_code &ec)
{
	// ec may be set if the timer is cancelled, i.e., updated
	if (! ec) {
		logger->log_error(name(), "No data received for too long, re-establishing connection");
		close_device();
	}
}
