
/***************************************************************************
 *  realsense_thread.cpp - realsense
 *
 *  Plugin created: Mon Jun 13 17:09:44 2016

 *  Copyright  2016  Johannes Rothe
 *             2017  Till Hofmann
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

#include "realsense2_thread.h"

#include <interfaces/SwitchInterface.h>

using namespace fawkes;

/** @class RealsenseThread 'realsense_thread.h' 
 * Driver for the Intel RealSense Camera providing Depth Data as Pointcloud
 * Inspired by Intel® RealSense™ Camera - F200 ROS Nodelet
 * @author Johannes Rothe
 */

Realsense2Thread::Realsense2Thread()
: Thread("Realsense2Thread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
  switch_if_(NULL)
{
}

void
Realsense2Thread::init()
{
	//set config values
	const std::string cfg_prefix = "/realsense/";
	frame_id_                    = config->get_string(cfg_prefix + "frame_id");
	pcl_id_                      = config->get_string(cfg_prefix + "pcl_id");
	laser_power_                 = config->get_int(cfg_prefix + "device_options/laser_power");
	restart_after_num_errors_ =
	  config->get_uint_or_default(std::string(cfg_prefix + "restart_after_num_errors").c_str(), 50);

	cfg_use_switch_ = config->get_bool_or_default((cfg_prefix + "use_switch").c_str(), true);

	if (cfg_use_switch_) {
		logger->log_info(name(), "Switch enabled");
	} else {
		logger->log_info(name(), "Switch will be ignored");
	}

    switch_if_ = blackboard->open_for_writing<SwitchInterface>("realsense2");
	switch_if_->set_enabled(true);
	switch_if_->write();

	camera_scale_ = 1;
	//initalize pointcloud
	realsense_depth_refptr_           = new Cloud();
	realsense_depth_                  = pcl_utils::cloudptr_from_refptr(realsense_depth_refptr_);
	realsense_depth_->header.frame_id = frame_id_;
	realsense_depth_->width           = 0;
	realsense_depth_->height          = 0;
	realsense_depth_->resize(0);
	pcl_manager->add_pointcloud(pcl_id_.c_str(), realsense_depth_refptr_);

	connect_and_start_camera();


}

void
Realsense2Thread::loop()
{

}

void
Realsense2Thread::finalize()
{

}

/* Create RS context and start the depth stream
 * @return true when succesfull
 */
bool
Realsense2Thread::connect_and_start_camera()
{

    return true;
}

/* Get the rs_device pointer and printout camera details
 * @return rs_device
 */
//rs2::device *
//Realsense2Thread::get_camera()
//{


//    return rs2::device();
//}

/*
 * Enable the depth stream from rs_device
 */
void
Realsense2Thread::enable_depth_stream()
{

}

/*
 * printout and free the rs_error if available
 */
void
Realsense2Thread::log_error()
{

}

/*
 * Testing function to log the depth pixel distancens
 */
void
Realsense2Thread::log_depths(const uint16_t *image)
{

}

/*
 * Stop the device and delete the context properly
 */
void
Realsense2Thread::stop_camera()
{

}

/**
 * Read the switch interface and start/stop the camera if necessary.
 * @return true iff the interface is currently enabled.
 */
bool
Realsense2Thread::read_switch()
{
	while (!switch_if_->msgq_empty()) {
		Message *msg = switch_if_->msgq_first();
		if (dynamic_cast<SwitchInterface::EnableSwitchMessage *>(msg)) {
			enable_camera_ = true;
		} else if (dynamic_cast<SwitchInterface::DisableSwitchMessage *>(msg)) {
			enable_camera_ = false;
		}
		switch_if_->msgq_pop();
	}
	switch_if_->set_enabled(enable_camera_);
	switch_if_->write();
	return switch_if_->is_enabled();
}
