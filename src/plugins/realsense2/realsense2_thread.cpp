
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

#include <interfaces/CameraControlInterface.h>
#include <interfaces/Realsense2Interface.h>
#include <interfaces/SwitchInterface.h>

using namespace fawkes;

/** @class Realsense2Thread 'realsense2_thread.h'
 * Driver for the Intel RealSense Camera providing Depth Data as Pointcloud
 * Inspired by realsense fawkes plugin
 * Provides functionality to save a camera frame into a PNG File
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
	rgb_path_ = config->get_string_or_default((cfg_prefix + "rgb_path").c_str(),
	                                          "/home/robotino/realsense_images/");
	//rgb camera resolution/frame rate
	//rgb_width_      = config->get_int_or_default((cfg_prefix + "rgb_width").c_str(), 1920);
	//rgb_height_     = config->get_int_or_default((cfg_prefix + "rgb_height").c_str(), 1080);
	rgb_frame_rate_ = config->get_int_or_default((cfg_prefix + "rgb_frame_rate").c_str(), 10);

	camera_if_name_ =
	  config->get_string_or_default((cfg_prefix + "camera_interface_name").c_str(), "realsense2_cam");

	if (cfg_use_switch_) {
		logger->log_info(name(), "Switch enabled");
	} else {
		logger->log_info(name(), "Switch will be ignored");
	}

	switch_if_ = blackboard->open_for_writing<SwitchInterface>(switch_if_name_.c_str());
	switch_if_->set_enabled(true);
	switch_if_->write();

	//CameraControlInterface
	camera_if_ = blackboard->open_for_writing<CameraControlInterface>(camera_if_name_.c_str());

	//Realsense2Interface
	rs_if_ = blackboard->open_for_writing<Realsense2Interface>("BoundingBoxInterface");

	camera_scale_ = 1;
	// initalize pointcloud
	realsense_depth_refptr_           = new Cloud();
	realsense_depth_                  = pcl_utils::cloudptr_from_refptr(realsense_depth_refptr_);
	realsense_depth_->header.frame_id = frame_id_;
	realsense_depth_->width           = 0;
	realsense_depth_->height          = 0;
	realsense_depth_->resize(0);
	pcl_manager->add_pointcloud(pcl_id_.c_str(), realsense_depth_refptr_);

	rs_pipe_     = new rs2::pipeline();
	rs_context_  = new rs2::context();
	rs_rgb_pipe_ = new rs2::pipeline();
}

void
Realsense2Thread::loop()
{
	if (!camera_running_) {
		camera_running_ = start_camera();
		return;
	}

	// take picture
	if (enable_camera_ && read_camera_control()) {
		if (rs_rgb_pipe_->poll_for_frames(&rs_rgb_data_)) {
			error_counter_               = 0;
			rs2::video_frame color_frame = rs_rgb_data_.first(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
			image_name_ = rgb_path_ + obj_name_ + color_frame.get_profile().stream_name() + ".png";
			png_writer_.set_filename(image_name_.c_str());
			png_writer_.set_dimensions(color_frame.get_width(), color_frame.get_height());
			png_writer_.set_buffer(firevision::RGB, (unsigned char *)color_frame.get_data());
			png_writer_.write();
			camera_if_->set_image_name(image_name_.c_str());
			camera_if_->set_msgid(camera_if_last_msgid_);
			camera_if_->write();
			logger->log_info(name(), "Saving image to %s", image_name_.c_str());
		} else {
			error_counter_++;
			logger->log_warn(name(), "Poll for rgb frames not successful ()");
			if (error_counter_ >= restart_after_num_errors_) {
				logger->log_warn(name(), "Polling failed, restarting device");
				error_counter_ = 0;
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

	// get bounding box
	if (read_bounding_box(bb)) {
		pixel_to_xyz();
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
	delete rs_pipe_;
	delete rs_context_;
	delete rs_rgb_pipe_;
	realsense_depth_refptr_.reset();
	pcl_manager->remove_pointcloud(pcl_id_.c_str());
	blackboard->close(switch_if_);
	blackboard->close(camera_if_);
	blackboard->close(rs_if_);
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
		rs_rgb_pipe_->stop();
	} catch (const std::exception &e) {
	}

	try {
		if (!get_camera(rs_device_)) {
			return false;
		}
		// depth config
		rs2::config config;
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

		//rgb config
		rs2::config rgb_config;
		rgb_width_  = intrinsics_.width;
		rgb_height_ = intrinsics_.height;
		rgb_config.enable_stream(
		  RS2_STREAM_COLOR, rgb_width_, rgb_height_, RS2_FORMAT_RGB8, rgb_frame_rate_);
		rs2::pipeline_profile rs_pipeline_profile_rgb_ = rs_rgb_pipe_->start(rgb_config);
		auto                  rgb_stream =
		  rs_pipeline_profile_rgb_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
		rgb_intrinsics_              = rgb_stream.get_intrinsics();
		rs2::color_sensor rgb_sensor = rs_device_.first<rs2::color_sensor>();
		logger->log_info(name(),
		                 "RGB Height: %d RGB Width: %d FPS: %d",
		                 rgb_intrinsics_.height,
		                 rgb_intrinsics_.width,
		                 rgb_frame_rate_);

		// 6 and 11 based on unique stream ids
		extrinsics_ =
		  sensor.get_stream_profiles()[6].get_extrinsics_to(rgb_sensor.get_stream_profiles()[11]);
		rgb_extrinsics_ =
		  rgb_sensor.get_stream_profiles()[11].get_extrinsics_to(sensor.get_stream_profiles()[6]);

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
	try {
		rs_rgb_pipe_->stop();
	} catch (const std::exception &e) {
	}
}

/*
 * Write XYZ to Interface from aligned depth frame
 */
void
Realsense2Thread::pixel_to_xyz()
{
	rs2::depth_frame tmp_frm = rs2::depth_frame::frame();
	align_rgb_to_depth(tmp_frm);

	float           x         = 0;
	float           y         = 0;
	float           z         = 0;
	int             counter   = 0;
	const uint16_t *image     = reinterpret_cast<const uint16_t *>(tmp_frm.get_data());
	float           center[2] = {static_cast<float>(bb[0]), static_cast<float>(bb[1])};
	// get average xyz from (bounding box/2), throw away xyz = 0,0,0
	int qwidth  = int(bb[2] / 4);
	int qheight = int(bb[3] / 4);
	for (int i_x = 0; i_x < qwidth; i_x++) {
		for (int i_y = 0; i_y < qheight; i_y++) {
			float depth_p0[2];
			float depth_p1[2];
			float depth_p2[2];
			float depth_p3[2];
			float xyz0[3];
			float xyz1[3];
			float xyz2[3];
			float xyz3[3];
			float pixel0[2] = {center[0] + i_x, center[1] + i_y};
			float pixel1[2] = {center[0] + i_x, center[1] - i_y};
			float pixel2[2] = {center[0] - i_x, center[1] + i_y};
			float pixel3[2] = {center[0] - i_x, center[1] - i_y};

			rs2_project_color_pixel_to_depth_pixel(depth_p0,
			                                       image,
			                                       camera_scale_,
			                                       0.0,
			                                       1.0,
			                                       &intrinsics_,
			                                       &rgb_intrinsics_,
			                                       &rgb_extrinsics_,
			                                       &extrinsics_,
			                                       pixel0);

			rs2_project_color_pixel_to_depth_pixel(depth_p1,
			                                       image,
			                                       camera_scale_,
			                                       0.0,
			                                       1.0,
			                                       &intrinsics_,
			                                       &rgb_intrinsics_,
			                                       &rgb_extrinsics_,
			                                       &extrinsics_,
			                                       pixel1);

			rs2_project_color_pixel_to_depth_pixel(depth_p2,
			                                       image,
			                                       camera_scale_,
			                                       0.0,
			                                       1.0,
			                                       &intrinsics_,
			                                       &rgb_intrinsics_,
			                                       &rgb_extrinsics_,
			                                       &extrinsics_,
			                                       pixel2);

			rs2_project_color_pixel_to_depth_pixel(depth_p3,
			                                       image,
			                                       camera_scale_,
			                                       0.0,
			                                       1.0,
			                                       &intrinsics_,
			                                       &rgb_intrinsics_,
			                                       &rgb_extrinsics_,
			                                       &extrinsics_,
			                                       pixel3);

			float scaled_depth0 = camera_scale_
			                      * image[intrinsics_.width * (static_cast<int>(center[1]) + i_y)
			                              + (static_cast<int>(center[0]) + i_x)];
			float scaled_depth1 = camera_scale_
			                      * image[intrinsics_.width * (static_cast<int>(center[1]) + i_y)
			                              + (static_cast<int>(center[0]) - i_x)];
			float scaled_depth2 = camera_scale_
			                      * image[intrinsics_.width * (static_cast<int>(center[1]) - i_y)
			                              + (static_cast<int>(center[0]) + i_x)];
			float scaled_depth3 = camera_scale_
			                      * image[intrinsics_.width * (static_cast<int>(center[1]) - i_y)
			                              + (static_cast<int>(center[0]) - i_x)];

			rs2_deproject_pixel_to_point(xyz0, &intrinsics_, pixel0, scaled_depth0);
			rs2_deproject_pixel_to_point(xyz1, &intrinsics_, pixel1, scaled_depth1);
			rs2_deproject_pixel_to_point(xyz2, &intrinsics_, pixel2, scaled_depth2);
			rs2_deproject_pixel_to_point(xyz3, &intrinsics_, pixel3, scaled_depth3);

			// only use depths between 12-20cm
			if (.12 < scaled_depth0 && scaled_depth0 < 0.2) {
				rs2_deproject_pixel_to_point(xyz0, &intrinsics_, depth_p0, scaled_depth0);
				if ((isfinite(xyz0[0]) && isfinite(xyz0[1]) && isfinite(xyz0[2]))
				    && ((xyz0[0] + xyz0[1] + xyz0[2]) != 0.0)) {
					x = x + xyz0[0];
					y = y + xyz0[1];
					z = z + xyz0[2];
					counter++;
				}
			}
			if (.12 < scaled_depth1 && scaled_depth1 < 0.2) {
				rs2_deproject_pixel_to_point(xyz1, &intrinsics_, depth_p1, scaled_depth1);
				if ((isfinite(xyz1[0]) && isfinite(xyz1[1]) && isfinite(xyz1[2]))
				    && ((xyz1[0] + xyz1[1] + xyz1[2]) != 0.0)) {
					x = x + xyz1[0];
					y = y + xyz1[1];
					z = z + xyz1[2];
					counter++;
				}
			}
			if (.12 < scaled_depth2 && scaled_depth2 < 0.2) {
				rs2_deproject_pixel_to_point(xyz2, &intrinsics_, depth_p2, scaled_depth2);
				if ((isfinite(xyz2[0]) && isfinite(xyz2[1]) && isfinite(xyz2[2]))
				    && ((xyz2[0] + xyz2[1] + xyz2[2]) != 0.0)) {
					x = x + xyz2[0];
					y = y + xyz2[1];
					z = z + xyz2[2];
					counter++;
				}
			}
			if (.12 < scaled_depth3 && scaled_depth3 < 0.2) {
				rs2_deproject_pixel_to_point(xyz3, &intrinsics_, depth_p3, scaled_depth3);
				if ((isfinite(xyz3[0]) && isfinite(xyz3[1]) && isfinite(xyz3[2]))
				    && ((xyz3[0] + xyz3[1] + xyz3[2]) != 0.0)) {
					x = x + xyz3[0];
					y = y + xyz3[1];
					z = z + xyz3[2];
					counter++;
				}
			}
		}
	}
	if (counter == 0)
		counter++;
	x = x / counter;
	y = y / counter;
	z = z / counter;
	rs_if_->set_translation(0, x);
	rs_if_->set_translation(1, y);
	rs_if_->set_translation(2, z);
	rs_if_->set_msgid(bb[4]);
	rs_if_->write();
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

/**
 * Read the CameraControl interface to take a picture.
 * @return new msg recevied
 */
bool
Realsense2Thread::read_camera_control()
{
	bool save = false;
	while (!camera_if_->msgq_empty()) {
		if (camera_if_->msgq_first_is<CameraControlInterface::SaveImageMessage>()) {
			CameraControlInterface::SaveImageMessage *msg =
			  camera_if_->msgq_first<CameraControlInterface::SaveImageMessage>();
			obj_name_             = std::string(msg->image_name());
			camera_if_last_msgid_ = msg->id();
			save                  = true;
		} else {
			logger->log_warn(name(), "Unknown message received");
		}
		camera_if_->msgq_pop();
	}
	return save;
}

/**
 * Read the Realsense2 interface to get the ROI to determine a XYZ pose.
 * @param bb bounding box
 * @return new_msg
 */
bool
Realsense2Thread::read_bounding_box(std::vector<int> &bb)
{
	bool new_msg = false;
	bb.clear();
	while (!rs_if_->msgq_empty()) {
		if (rs_if_->msgq_first_is<Realsense2Interface::BoundingBoxMessage>()) {
			Realsense2Interface::BoundingBoxMessage *msg =
			  rs_if_->msgq_first<Realsense2Interface::BoundingBoxMessage>();
			bb.push_back(msg->center_x());
			bb.push_back(msg->center_y());
			bb.push_back(msg->width());
			bb.push_back(msg->height());
			bb.push_back(msg->id());
			new_msg = true;
		} else {
			logger->log_warn(name(), "Unknown message received");
		}
		rs_if_->msgq_pop();
	}
	return new_msg;
}

/**
 * Align the depth and color stream to get the 3D coordinates from pixels
 * @return aligned depth frame
 */
void
Realsense2Thread::align_rgb_to_depth(rs2::depth_frame &input_frame)
{
	rs2::align    align(RS2_STREAM_COLOR);
	rs2::frameset frameset = rs2::frameset();
	while (not rs_pipe_->poll_for_frames(&frameset)) {}
	auto processed = align.process(frameset);
	input_frame    = processed.get_depth_frame();
}
