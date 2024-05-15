
/***************************************************************************
 *  realsense2_plugin.cpp - realsense2
 *
 *  Plugin created: Wed May 22 10:09:22 2019
 *
 *  Copyright  2019 Christoph Gollok
 *             2022 Matteo Tschesche
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
	image_width_    = config->get_int_or_default((cfg_prefix + "rgb_width").c_str(), 640);
	image_height_   = config->get_int_or_default((cfg_prefix + "rgb_height").c_str(), 480);
	rgb_frame_rate_ = config->get_int_or_default((cfg_prefix + "frame_rate").c_str(), 30);
	save_images_    = config->get_bool_or_default((cfg_prefix + "save_images").c_str(), false);

	serial_no_ = config->get_string_or_default((cfg_prefix + "serial_no").c_str(), "");

	if (cfg_use_switch_) {
		logger->log_info(name(), "Switch enabled");
	} else {
		logger->log_info(name(), "Switch will be ignored");
	}

	switch_if_ = blackboard->open_for_writing<SwitchInterface>(switch_if_name_.c_str());
	switch_if_->set_enabled(true);
	switch_if_->write();

	camera_scale_ = 1;
	// initalize pointcloud
	realsense_depth_refptr_           = new Cloud();
	realsense_depth_                  = pcl_utils::cloudptr_from_refptr(realsense_depth_refptr_);
	realsense_depth_->header.frame_id = frame_id_;
	realsense_depth_->width           = 0;
	realsense_depth_->height          = 0;
	realsense_depth_->resize(0);
	pcl_manager->add_pointcloud(pcl_id_.c_str(), realsense_depth_refptr_);

	rs_pipe_    = new rs2::pipeline();
	rs_context_ = new rs2::context();

	shm_id_ = config->get_string((cfg_prefix + "shm_image_id").c_str());

	rgb_rs_pipe_ = new rs2::pipeline();

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
		if (rgb_rs_pipe_->poll_for_frames(&rgb_rs_data_)) {
			rgb_error_counter_           = 0;
			rs2::video_frame color_frame = rgb_rs_data_.first(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
			fawkes::Time     now(clock);

			// set image in shared memory
			firevision::convert(firevision::RGB,
			                    firevision::RGB,
			                    (unsigned char *)color_frame.get_data(),
			                    shm_buffer_->buffer(),
			                    image_width_,
			                    image_height_);
			shm_buffer_->set_capture_time(&now);

			if (save_images_) {
				image_name_ =
				  rgb_path_ + std::to_string(name_it_) + color_frame.get_profile().stream_name() + ".png";
				png_writer_.set_filename(image_name_.c_str());
				png_writer_.set_dimensions(color_frame.get_width(), color_frame.get_height());
				png_writer_.set_buffer(firevision::RGB, (unsigned char *)color_frame.get_data());
				png_writer_.write();
				name_it_++;
			}
		} else {
			rgb_error_counter_++;
			if (rgb_error_counter_ >= restart_after_num_errors_) {
				logger->log_warn(name(), "Polling failed, restarting device");
				rgb_error_counter_ = 0;
				stop_camera();
				start_camera();
			}
		}
	}

	if (cfg_use_switch_) {
		read_switch();
	}

	if (enable_camera_ && !depth_enabled_) {
		enable_depth_stream();
		return;
	} else if (!enable_camera_ && depth_enabled_) {
		disable_depth_stream();
		return;
	} else if (!depth_enabled_) {
		return;
	}
	if (rs_pipe_->poll_for_frames(&rs_data_)) {
		rs2::frame depth_frame = rs_data_.first(RS2_STREAM_DEPTH);
		error_counter_         = 0;
		const uint16_t *image  = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
		Cloud::iterator it     = realsense_depth_->begin();
		for (int y = 0; y < intrinsics_.height; y++) {
			for (int x = 0; x < intrinsics_.width; x++) {
				float scaled_depth = camera_scale_ * (static_cast<float>(*image));
				float depth_point[3];
				float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
				rs2_deproject_pixel_to_point(depth_point, &intrinsics_, depth_pixel, scaled_depth);
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
		logger->log_warn(name(), "Poll for frames not successful ()");
		if (error_counter_ >= restart_after_num_errors_) {
			logger->log_warn(name(), "Polling failed, restarting device");
			error_counter_ = 0;
			stop_camera();
			start_camera();
		}
	}
}

void
Realsense2Thread::finalize()
{
	stop_camera();
	delete rgb_rs_pipe_;
	delete rs_pipe_;
	delete rs_context_;
	realsense_depth_refptr_.reset();
	pcl_manager->remove_pointcloud(pcl_id_.c_str());
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
		rgb_rs_pipe_->stop();
	} catch (const std::exception &e) {
	}

	try {
		if (!get_camera(rs_device_)) {
			return false;
		}
		rs2::config config;
		config.enable_device(rs_device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		config.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16, frame_rate_);
		rs2::pipeline_profile rs_pipeline_profile_ = rs_pipe_->start(config);
		auto                  depth_stream =
		  rs_pipeline_profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
		intrinsics_              = depth_stream.get_intrinsics();
		realsense_depth_->width  = intrinsics_.width;
		realsense_depth_->height = intrinsics_.height;
		realsense_depth_->resize(intrinsics_.width * intrinsics_.height);
		rs2::depth_sensor sensor = rs_device_.first<rs2::depth_sensor>();
		camera_scale_            = sensor.get_depth_scale();
		logger->log_info(name(),
		                 "Height: %d Width: %d Scale: %f FPS: %d",
		                 intrinsics_.height,
		                 intrinsics_.width,
		                 camera_scale_,
		                 frame_rate_);

		rs2::config rgb_rs_config;
		rgb_rs_config.enable_device(rs_device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		rgb_rs_config.enable_stream(
		  RS2_STREAM_COLOR, image_width_, image_height_, RS2_FORMAT_RGB8, rgb_frame_rate_);
		rs2::pipeline_profile rgb_rs_pipeline_profile_ = rgb_rs_pipe_->start(rgb_rs_config);
		auto                  rgb_stream =
		  rgb_rs_pipeline_profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
		rgb_intrinsics_              = rgb_stream.get_intrinsics();
		rs2::color_sensor rgb_sensor = rs_device_.first<rs2::color_sensor>();
		if (rgb_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
			// Enable auto-exposure
			rgb_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1); // 1 to enable, 0 to disable
		} else {
			logger->log_info(name(), "Auto-exposure not supported on this device");
			// Set exposure time
			if (rgb_sensor.supports(RS2_OPTION_EXPOSURE)) {
				rgb_sensor.set_option(RS2_OPTION_EXPOSURE, 600);
			} else {
				logger->log_info(name(), "Exposure control not supported on this device");
				std::cerr << "Exposure control not supported on this device" << std::endl;
			}
		}
		// Increase gain if needed
		if (rgb_sensor.supports(RS2_OPTION_GAIN)) {
			rgb_sensor.set_option(RS2_OPTION_GAIN, 50.0);
		}
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
		                                                      firevision::RGB,
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
			for (size_t i = 0; i < devlist.size(); i++) {
				if (devlist[i].is<rs400::advanced_mode>()) {
					dev = devlist[i].as<rs400::advanced_mode>();
				} else {
					dev = devlist[i];
				}

				std::string dev_name = "Unknown Device";
				if (dev.supports(RS2_CAMERA_INFO_NAME)) {
					dev_name = dev.get_info(RS2_CAMERA_INFO_NAME);
				} else {
					logger->log_info(name(), "RS2Option RS2_CAMERA_INFO_NAME not supported %d", 1);
				}

				if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
					std::string dev_sn = "########";
					dev_sn = std::string("#") + rs_device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
					logger->log_info(name(), "found device with serial number: %s", dev_sn.c_str());
					if (strcmp(rs_device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), serial_no_.c_str()) == 0) {
						logger->log_info(name(), "matched device with serial number: %s", dev_sn.c_str());
						logger->log_info(name(), "Camera Name: %s, SN: %s", dev_name.c_str(), dev_sn.c_str());
						return true;
					}
				} else {
					logger->log_info(name(), "RS2Option RS2_CAMERA_INFO_SERIAL_NUMBER not supported");
				}
			}
			logger->log_warn(name(),
			                 "No device with serial number %s found, returning last device in list",
			                 serial_no_.c_str());
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
 * Enable the depth stream from rs_device
 */
void
Realsense2Thread::enable_depth_stream()
{
	logger->log_info(name(), "Enable depth Stream");

	try {
		rs2::depth_sensor depth_sensor = rs_device_.first<rs2::depth_sensor>();
		if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
			depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,
			                        1.f); // Enable emitter
			depth_enabled_ = true;
		} else if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
			if (laser_power_ == -1) {
				rs2::option_range range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
				laser_power_            = range.max;
			}
			logger->log_info(name(), "Enable depth stream with Laser Power: %f", laser_power_);
			depth_sensor.set_option(RS2_OPTION_LASER_POWER, laser_power_); // Set max power
			depth_enabled_ = true;
		} else {
			logger->log_warn(name(), "Enable depth stream not supported on device");
		}

	} catch (const rs2::error &e) {
		logger->log_error(name(),
		                  "RealSense error calling %s ( %s ):\n    %s",
		                  e.get_failed_function().c_str(),
		                  e.get_failed_args().c_str(),
		                  e.what());
		return;
	} catch (const std::exception &e) {
		logger->log_error(name(), "%s", e.what());
		return;
	}
}

/*
 * Disable the depth stream from rs_device
 */
void
Realsense2Thread::disable_depth_stream()
{
	logger->log_info(name(), "Disable Depth Stream");

	try {
		rs2::depth_sensor depth_sensor = rs_device_.first<rs2::depth_sensor>();
		if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
			depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,
			                        0.f); // Disable emitter
			depth_enabled_ = false;
		} else if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
			rs2::option_range range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
			depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.min); // Set max power
			depth_enabled_ = false;
		} else {
			logger->log_warn(name(), "Disable depth stream not supported on device");
		}
	} catch (const rs2::error &e) {
		logger->log_error(name(),
		                  "RealSense error calling %s ( %s ):\n    %s",
		                  e.get_failed_function().c_str(),
		                  e.get_failed_args().c_str(),
		                  e.what());
		return;
	} catch (const std::exception &e) {
		logger->log_error(name(), "%s", e.what());
		return;
	}
}

/*
 * Stop the device and delete the context properly
 */
void
Realsense2Thread::stop_camera()
{
	camera_running_ = false;
	depth_enabled_  = false;
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
