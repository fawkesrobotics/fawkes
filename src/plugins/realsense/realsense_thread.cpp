
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

#include "realsense_thread.h"

#include <interfaces/SwitchInterface.h>

using namespace fawkes;

/** @class RealsenseThread 'realsense_thread.h' 
 * Driver for the Intel RealSense Camera providing Depth Data as Pointcloud
 * Inspired by Intel® RealSense™ Camera - F200 ROS Nodelet
 * @author Johannes Rothe
 */

RealsenseThread::RealsenseThread()
: Thread("RealsenseThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
  switch_if_(NULL)
{
}

void
RealsenseThread::init()
{
	//set config values
	const std::string cfg_prefix = "/realsense/";
	frame_id_                    = config->get_string(cfg_prefix + "frame_id");
	pcl_id_                      = config->get_string(cfg_prefix + "pcl_id");
    laser_power_high_                 = config->get_int(cfg_prefix + "device_options/laser_power_high");
    laser_power_low_                 = config->get_int(cfg_prefix + "device_options/laser_power_low");
	restart_after_num_errors_ =
	  config->get_uint_or_default(std::string(cfg_prefix + "restart_after_num_errors").c_str(), 50);

	cfg_use_switch_ = config->get_bool_or_default((cfg_prefix + "use_switch").c_str(), true);

	if (cfg_use_switch_) {
		logger->log_info(name(), "Switch enabled");
	} else {
		logger->log_info(name(), "Switch will be ignored");
	}

	switch_if_ = blackboard->open_for_writing<SwitchInterface>("realsense");
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

	rs_stream_type_ = RS_STREAM_DEPTH;
	connect_and_start_camera();
}

void
RealsenseThread::loop()
{
	if (cfg_use_switch_) {
		read_switch();
	}
    if (enable_camera_){
        if (!camera_running_){
            connect_and_start_camera();
            // Start reading in the next loop
        }else{
            // camera already runing. Increase laser power
            set_laser_power(laser_power_high_);
        }
		return;
	} else if (!enable_camera_) {
		if (camera_running_) {
            // decrease laser power
            set_laser_power(laser_power_high_);
		}
		return;
	}
	if (rs_poll_for_frames(rs_device_, &rs_error_) == 1) {
		error_counter_ = 0;
		const uint16_t *image =
		  reinterpret_cast<const uint16_t *>(rs_get_frame_data(rs_device_, rs_stream_type_, NULL));
		log_error();
		Cloud::iterator it = realsense_depth_->begin();
		for (int y = 0; y < z_intrinsic_.height; y++) {
			for (int x = 0; x < z_intrinsic_.width; x++) {
				float scaled_depth = camera_scale_ * ((float)*image);
				float depth_point[3];
				float depth_pixel[2] = {(float)x, (float)y};
				rs_deproject_pixel_to_point(depth_point, &z_intrinsic_, depth_pixel, scaled_depth);
				it->x = depth_point[0];
				it->y = depth_point[1];
				it->z = depth_point[2];
				++image;
				++it;
			}
		}
		pcl_utils::set_time(realsense_depth_refptr_, fawkes::Time(clock));
	} else {
		error_counter_++;
		logger->log_warn(name(),
		                 "Poll for frames not successful (%s)",
		                 rs_get_error_message(rs_error_));
		if (error_counter_ >= restart_after_num_errors_) {
			logger->log_warn(name(), "Polling failed, restarting device");
			error_counter_ = 0;
			stop_camera();
			connect_and_start_camera();
		}
	}
}

void
RealsenseThread::finalize()
{
	realsense_depth_refptr_.reset();
	pcl_manager->remove_pointcloud(pcl_id_.c_str());
	stop_camera();
	blackboard->close(switch_if_);
	//TODO Documentation with librealsense
}

/* Create RS context and start the depth stream
 * @return true when succesfull
 */
bool
RealsenseThread::connect_and_start_camera()
{
	rs_context_ = rs_create_context(RS_API_VERSION, &rs_error_);
	log_error();
	num_of_cameras_ = rs_get_device_count(rs_context_, &rs_error_);
	logger->log_info(name(), "No. of cameras: %i ", num_of_cameras_);
	if (num_of_cameras_ < 1) {
		logger->log_error(name(), "No camera detected!");
		rs_delete_context(rs_context_, &rs_error_);
		camera_running_ = false;
		return camera_running_;
	}

	rs_device_ = get_camera();
    rs_set_device_option(rs_device_, RS_OPTION_F200_LASER_POWER, laser_power_high_, &rs_error_);
	log_error();
	enable_depth_stream();

	rs_start_device(rs_device_, &rs_error_);
	log_error();

	logger->log_info(name(),
	                 "Stream format: %s",
	                 rs_format_to_string(
	                   rs_get_stream_format(rs_device_, rs_stream_type_, &rs_error_)));

	camera_running_ = true;
	camera_scale_   = rs_get_device_depth_scale(rs_device_, NULL);
	rs_get_stream_intrinsics(rs_device_, rs_stream_type_, &z_intrinsic_, &rs_error_);
	realsense_depth_->width  = z_intrinsic_.width;
	realsense_depth_->height = z_intrinsic_.height;
	realsense_depth_->resize(z_intrinsic_.width * z_intrinsic_.height);
	logger->log_info(name(), "Height: %i, Width: %i", z_intrinsic_.height, z_intrinsic_.width);
	return camera_running_;
}

/* Get the rs_device pointer and printout camera details
 * @return rs_device
 */
rs_device *
RealsenseThread::get_camera()
{
	//assume we only have one camera connected so take index 0
	rs_device *rs_detected_device = rs_get_device(rs_context_, 0, &rs_error_);
	//print device details
	logger->log_info(name(),
	                 "\n\nDetected Device:\n"
	                 "Serial No: %s\n"
	                 "Firmware %s\n"
	                 "Name %s\n"
	                 "USB Port ID %s\n",
	                 rs_get_device_serial(rs_detected_device, &rs_error_),
	                 rs_get_device_firmware_version(rs_detected_device, &rs_error_),
	                 rs_get_device_name(rs_detected_device, &rs_error_),
	                 rs_get_device_usb_port_id(rs_detected_device, &rs_error_));
	log_error();
	return rs_detected_device;
}

/*
 * Enable the depth stream from rs_device
 */
void
RealsenseThread::enable_depth_stream()
{
	rs_enable_stream_preset(rs_device_, rs_stream_type_, RS_PRESET_BEST_QUALITY, &rs_error_);
	log_error();
	if (rs_is_stream_enabled(rs_device_, rs_stream_type_, &rs_error_)) {
		logger->log_info(name(),
		                 "Depth stream enabled! Streaming with %i fps",
		                 rs_get_stream_framerate(rs_device_, rs_stream_type_, &rs_error_));
		log_error();
	} else {
		log_error();
		throw Exception("Couldn't start depth stream! Stream type: %s",
		                rs_stream_to_string(rs_stream_type_));
	}
}

/*
 * Set Laser Power from rs_device
 */

void
RealsenseThread::set_laser_power(int laser_power){
    rs_device_ = get_camera();
    rs_set_device_option(rs_device_, RS_OPTION_F200_LASER_POWER, laser_power_high_, &rs_error_);
    log_error();
    logger->log_info(name(),
                     "set laser power to %i ",
                     laser_power);
}

/*
 * printout and free the rs_error if available
 */
void
RealsenseThread::log_error()
{
	if (rs_error_) {
		logger->log_warn(name(), "Realsense Error: %s", rs_get_error_message(rs_error_));
		rs_free_error(rs_error_);
		rs_error_ = nullptr;
	}
}

/*
 * Testing function to log the depth pixel distancens
 */
void
RealsenseThread::log_depths(const uint16_t *image)
{
	std::string out;
	for (uint16_t i = 0; i < rs_get_stream_height(rs_device_, rs_stream_type_, NULL); i++) {
		for (uint16_t i = 0; i < rs_get_stream_width(rs_device_, rs_stream_type_, NULL); i++) {
			float depth_in_meters = camera_scale_ * image[i];
			out += std::to_string(depth_in_meters) + " ";
		}
		out += "\n";
	}
	logger->log_info(name(), "%s\n\n\n\n\n", out.c_str());
}

/*
 * Stop the device and delete the context properly
 */
void
RealsenseThread::stop_camera()
{
	if (camera_running_) {
		logger->log_info(name(), "Stopping realsense camera ...");
		rs_stop_device(rs_device_, &rs_error_);
		rs_delete_context(rs_context_, &rs_error_);
		log_error();
		logger->log_info(name(), "Realsense camera stopped!");
		camera_running_ = false;
	}
}

/**
 * Read the switch interface and start/stop the camera if necessary.
 * @return true iff the interface is currently enabled.
 */
bool
RealsenseThread::read_switch()
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
