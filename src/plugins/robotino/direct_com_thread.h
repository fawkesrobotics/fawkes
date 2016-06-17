/***************************************************************************
 *  direct_com_thread.h - Robotino com thread for direct communication
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

#ifndef __PLUGINS_ROBOTINO_DIRECT_COM_THREAD_H_
#define __PLUGINS_ROBOTINO_DIRECT_COM_THREAD_H_

#include "com_thread.h"
#include "direct_com_message.h"
#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <utils/time/time.h>

#include <memory>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>

class DirectRobotinoComMessage;

namespace fawkes {
	class Mutex;
	class TimeWait;

	class BatteryInterface;
	class RobotinoSensorInterface;
	class IMUInterface;
}

class DirectRobotinoComThread
: public RobotinoComThread,
	public fawkes::ConfigurableAspect
{
 public:
	DirectRobotinoComThread();
	virtual ~DirectRobotinoComThread();

	virtual void init();
	virtual void once();
	virtual void loop();
	bool    prepare_finalize_user();

	virtual void finalize();

	virtual bool is_connected();

	virtual void set_gripper(bool opened);
	virtual bool is_gripper_open();
	virtual void set_speed_points(float s1, float s2, float s3);
	virtual void get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t);
	virtual void get_odometry(double &x, double &y, double &phi);

	virtual void reset_odometry();
	virtual void set_bumper_estop_enabled(bool enabled);
	virtual void set_motor_accel_limits(float min_accel, float max_accel);
	virtual void set_digital_output(unsigned int digital_out, bool enable);

	virtual void set_desired_vel(float vx, float vy, float omega);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
	std::string find_device_udev();
	bool find_controld3();
	void open_device(bool wait_replies);
	void close_device();
	void flush_device();
	void check_deadline();
	void request_data();
	void handle_request_data(const boost::system::error_code &ec);
	void handle_nodata(const boost::system::error_code &ec);
	void update_nodata_timer();

	void drive();
	void handle_drive(const boost::system::error_code &ec);

	DirectRobotinoComMessage::pointer read_packet();
	void send_message(DirectRobotinoComMessage &msg);
	DirectRobotinoComMessage::pointer
		send_and_recv(DirectRobotinoComMessage &msg);
	void process_message(DirectRobotinoComMessage::pointer m);

 private:
	std::string     cfg_device_;
	bool            cfg_enable_gyro_;
	unsigned int    cfg_sensor_update_cycle_time_;
	bool            cfg_gripper_enabled_;
	float           cfg_rpm_max_;
	unsigned int    cfg_nodata_timeout_;
	unsigned int    cfg_drive_update_interval_;
	unsigned int    cfg_read_timeout_;
	bool            cfg_log_checksum_errors_;
	unsigned int    cfg_checksum_error_recover_;
	unsigned int    cfg_checksum_error_critical_;
	
	bool opened_;
	unsigned int open_tries_;

	unsigned int checksum_errors_;

	uint8_t         digital_outputs_;

	boost::asio::io_service       io_service_;
	boost::asio::serial_port      serial_;
	boost::asio::io_service::work io_service_work_;
	boost::asio::deadline_timer   deadline_;
	boost::asio::streambuf        input_buffer_;
	boost::mutex                  io_mutex_;

	boost::asio::deadline_timer   request_timer_;
	boost::asio::deadline_timer   nodata_timer_;
	boost::asio::deadline_timer   drive_timer_;
	
};


#endif

