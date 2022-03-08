
/***************************************************************************
 *  realsense2_plugin.cpp - realsense2
 *
 *  Plugin created: Wed May 22 10:09:22 2019
 *
 *  Copyright  2019 Christoph Gollok
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

/** @class Realsense2Thread 'realsense2_thread.h'
 * Driver for the Intel RealSense Camera providing Depth Data as Pointcloud
 * Inspired by realsense fawkes plugin
 * @author Christoph Gollok
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
	// set config values
	const std::string cfg_prefix = "/realsense2/";
	frame_id_ = config->get_string_or_default((cfg_prefix + "frame_id").c_str(), "cam_conveyor");
	pcl_id_ = config->get_string_or_default((cfg_prefix + "pcl_id").c_str(), "/camera/depth/points");
	switch_if_name_ =
	  config->get_string_or_default((cfg_prefix + "switch_if_name").c_str(), "realsense2");
	restart_after_num_errors_ =
	  config->get_uint_or_default((cfg_prefix + "restart_after_num_errors").c_str(), 50);
	frame_rate_  = config->get_uint_or_default((cfg_prefix + "frame_rate").c_str(), 30);
	laser_power_ = config->get_float_or_default((cfg_prefix + "laser_power").c_str(), -1);

	cfg_use_switch_ = config->get_bool_or_default((cfg_prefix + "use_switch").c_str(), true);

	//rgb image path
	rgb_path_ =
	  config->get_string_or_default((cfg_prefix + "rgb_path").c_str(), "/tmp/realsense_images/");
	//rgb camera resolution/frame rate
	image_width_  = config->get_int_or_default((cfg_prefix + "rgb_width").c_str(), 640);
	image_height_ = config->get_int_or_default((cfg_prefix + "rgb_height").c_str(), 480);
	frame_rate_   = config->get_int_or_default((cfg_prefix + "frame_rate").c_str(), 30);
	save_images_  = config->get_bool_or_default((cfg_prefix + "save_images").c_str(), false);

	if (cfg_use_switch_) {
		logger->log_info(name(), "Switch enabled");
	} else {
		logger->log_info(name(), "Switch will be ignored");
	}

	switch_if_ = blackboard->open_for_writing<SwitchInterface>(switch_if_name_.c_str());
	switch_if_->set_enabled(true);
	switch_if_->write();

	shm_id_ = config->get_string((cfg_prefix + "shm_image_id").c_str());

	camera_scale_ = 1;
	rs_context_   = new rs2::context();
	rs_pipe_      = new rs2::pipeline();

	name_it_ = 0;
}

void
Realsense2Thread::loop()
{
	if (!camera_running_) {
		camera_running_ = start_camera();
		return;
	}
	if (cfg_use_switch_) {
		read_switch();
	}

	// take picture
	if (enable_camera_) {
		if (rs_pipe_->poll_for_frames(&rs_data_)) {
			error_counter_               = 0;
			rs2::video_frame color_frame = rs_data_.first(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);

			if (save_images_) {
				image_name_ =
				  rgb_path_ + std::to_string(name_it_) + color_frame.get_profile().stream_name() + ".png";
				png_writer_.set_filename(image_name_.c_str());
				png_writer_.set_dimensions(color_frame.get_width(), color_frame.get_height());
				png_writer_.set_buffer(firevision::RGB, (unsigned char *)color_frame.get_data());
				png_writer_.write();
				name_it_++;
			}

			// set image in shared memory
			firevision::convert(firevision::RGB,
			                    firevision::BGR,
			                    (unsigned char *)color_frame.get_data(),
			                    shm_buffer_->buffer(),
			                    image_width_,
			                    image_height_);
		} else {
			error_counter_++;
			if (error_counter_ >= restart_after_num_errors_) {
				logger->log_warn(name(), "Polling failed, restarting device");
				error_counter_ = 0;
				stop_camera();
				start_camera();
			}
		}
	}
}

void
Realsense2Thread::finalize()
{
	stop_camera();
	delete rs_context_;
	delete rs_pipe_;
	blackboard->close(switch_if_);
}

/* Create RS context and start the depth stream
 * @return true when succesfull
 */
bool
Realsense2Thread::start_camera()
{
	try {
		rs_pipe_->stop();
	} catch (const std::exception &e) {
	}

	try {
		if (!get_camera(rs_device_)) {
			return false;
		}

		rs2::config rs_config;
		rs_config.enable_stream(
		  RS2_STREAM_COLOR, image_width_, image_height_, RS2_FORMAT_RGB8, frame_rate_);
		rs2::pipeline_profile rs_pipeline_profile_ = rs_pipe_->start(rs_config);
		auto                  rgb_stream =
		  rs_pipeline_profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
		intrinsics_                  = rgb_stream.get_intrinsics();
		rs2::color_sensor rgb_sensor = rs_device_.first<rs2::color_sensor>();
		logger->log_info(name(),
		                 "RGB Height: %d RGB Width: %d FPS: %d PPX: %f PPY: %f FX: %f FY: %f MODEL: %i "
		                 "COEFFS: %f %f %f %f %f",
		                 intrinsics_.height,
		                 intrinsics_.width,
		                 frame_rate_,
		                 intrinsics_.ppx,
		                 intrinsics_.ppy,
		                 intrinsics_.fx,
		                 intrinsics_.fy,
		                 intrinsics_.model,
		                 intrinsics_.coeffs[0],
		                 intrinsics_.coeffs[1],
		                 intrinsics_.coeffs[2],
		                 intrinsics_.coeffs[3],
		                 intrinsics_.coeffs[4]);
		shm_buffer_ = new firevision::SharedMemoryImageBuffer(shm_id_.c_str(),
		                                                      firevision::BGR,
		                                                      image_width_,
		                                                      image_height_);

		return true;

	} catch (const rs2::error &e) {
		logger->log_error(name(),
		                  "RealSense error calling %s ( %s ):\n    %s",
		                  e.get_failed_function().c_str(),
		                  e.get_failed_args().c_str(),
		                  e.what());
	} catch (const std::exception &e) {
		logger->log_error(name(), "%s", e.what());
	}

	return false;
}

/*
 * Get the rs_device pointer and printout camera details
 */
bool
Realsense2Thread::get_camera(rs2::device &dev)
{
	dev = nullptr;
	try {
		rs2::device_list devlist = rs_context_->query_devices();
		if (devlist.size() == 0) {
			logger->log_warn(name(), "No device connected, please connect a RealSense device");
			return false;
		} else {
			logger->log_info(name(), "found devices: %d", devlist.size());
			if (devlist.front().is<rs400::advanced_mode>()) {
				dev = devlist.front().as<rs400::advanced_mode>();
			} else {
				dev = devlist.front();
			}

			std::string dev_name = "Unknown Device";
			if (dev.supports(RS2_CAMERA_INFO_NAME)) {
				dev_name = dev.get_info(RS2_CAMERA_INFO_NAME);
			} else {
				logger->log_info(name(), "RS2Option RS2_CAMERA_INFO_NAME not supported %d", 1);
			}

			std::string dev_sn = "########";
			if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
				dev_sn = std::string("#") + rs_device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			} else {
				logger->log_info(name(), "RS2Option RS2_CAMERA_INFO_SERIAL_NUMBER not supported");
			}
			logger->log_info(name(), "Camera Name: %s, SN: %s", dev_name.c_str(), dev_sn.c_str());
			return true;
		}
	} catch (const rs2::error &e) {
		logger->log_error(name(),
		                  "RealSense error calling %s ( %s ):\n    %s",
		                  e.get_failed_function().c_str(),
		                  e.get_failed_args().c_str(),
		                  e.what());
		return false;
	} catch (const std::exception &e) {
		logger->log_error(name(), "%s", e.what());
		return false;
	}
}

/*
 * Stop the device and delete the context properly
 */
void
Realsense2Thread::stop_camera()
{
	camera_running_ = false;
	try {
		rs_pipe_->stop();
	} catch (const std::exception &e) {
	}
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
