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
#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <utils/time/time.h>

#include <memory>
#include <boost/asio.hpp>

class DirectRobotinoComMessage;

namespace fawkes {
	class Mutex;
	class Clock;
	class TimeWait;

	class BatteryInterface;
	class RobotinoSensorInterface;
	class IMUInterface;
}

class DirectRobotinoComThread
: public RobotinoComThread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::ClockAspect,
	public fawkes::BlackBoardAspect
{
 public:
	DirectRobotinoComThread();
	virtual ~DirectRobotinoComThread();

	virtual void init();
	virtual void once();
	virtual void loop();
	virtual void finalize();

	void update_bb_sensor();

	bool is_connected();

	void set_gripper(bool opened);
	bool is_gripper_open();
	void set_speed_points(float s1, float s2, float s3);
	void get_act_velocity(float &a1, float &a2, float &a3, unsigned int &seq, fawkes::Time &t);
	void get_odometry(double &x, double &y, double &phi);
	void reset_odometry();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
	std::string find_device_udev();
	void open_device();
	void close_device();
	void check_deadline();

	void process_sensor_msgs();
	void process_sensor_state();
	void process_com();
	void update_distances(float *voltages);

	void read_packet();
	void send_message(DirectRobotinoComMessage &msg);
	std::shared_ptr<DirectRobotinoComMessage>
		send_and_recv(DirectRobotinoComMessage &msg);

 private:
	std::string     cfg_device_;
	std::string     cfg_hostname_;
	bool            cfg_quit_on_disconnect_;
	bool            cfg_enable_gyro_;
	std::string     cfg_imu_iface_id_;
	unsigned int    cfg_sensor_update_cycle_time_;
	bool            cfg_gripper_enabled_;

	// Voltage to distance data points
	std::vector<std::pair<double, double> > voltage_to_dist_dps_;

	fawkes::Mutex    *data_mutex_;
	bool              new_data_;
	fawkes::TimeWait *time_wait_;
	unsigned int      last_seqnum_;

	fawkes::BatteryInterface        *batt_if_;
	fawkes::RobotinoSensorInterface *sens_if_;
	fawkes::IMUInterface            *imu_if_;

	boost::asio::io_service       io_service_;
	boost::asio::serial_port      serial_;
	boost::asio::io_service::work io_service_work_;
	boost::asio::deadline_timer   deadline_;
	boost::asio::streambuf        input_buffer_;

};


#endif

