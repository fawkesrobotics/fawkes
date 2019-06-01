
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
    const std::string cfg_prefix = "/realsense2/";
	frame_id_                    = config->get_string(cfg_prefix + "frame_id");
	pcl_id_                      = config->get_string(cfg_prefix + "pcl_id");
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

    rs_pipe_ = new rs2::pipeline();

	connect_and_start_camera();

//    std::thread wait_for_frames_thread([&]() {
//        if (camera_running_){
//            rs_data_= rs_pipe_->wait_for_frames(); // Wait for next set of frames from the camera
//            std::cout << "NEW SET OF FRAMES AVAILABLE" << std::endl;
//            frames_avalialble_ = true;
//        }
//    });


}

void
Realsense2Thread::loop()
{

    if (cfg_use_switch_) {
        read_switch();
    }
    if (enable_camera_ && !camera_running_) {
        connect_and_start_camera();
        // Start reading in the next loop
        return;
    } else if (!enable_camera_) {
        if (camera_running_) {
            stop_camera();
        }
        return;
    }


    if (rs_pipe_->poll_for_frames(&rs_data_))
    {
        rs2::frame depth_frame = rs_data_.first(RS2_STREAM_DEPTH);
        logger->log_info(name(),
                         "GOT RS2 DEPTH FRAME");
        error_counter_ = 0;
        const uint16_t *image =
          reinterpret_cast<const uint16_t *>(depth_frame.get_data());
        log_error();
        Cloud::iterator it = realsense_depth_->begin();
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
    }else {
        error_counter_++;
        logger->log_warn(name(),
                         "Poll for frames not successful ()");
        if (error_counter_ >= restart_after_num_errors_) {
            logger->log_warn(name(), "Polling failed, restarting device");
            error_counter_ = 0;
            stop_camera();
            connect_and_start_camera();
        }
    }
}

void
Realsense2Thread::finalize()
{
    delete rs_pipe_;
    delete rs_context_;
    realsense_depth_refptr_.reset();
    pcl_manager->remove_pointcloud(pcl_id_.c_str());
    stop_camera();
    blackboard->close(switch_if_);
}

/* Create RS context and start the depth stream
 * @return true when succesfull
 */
bool
Realsense2Thread::connect_and_start_camera()
{
    try {
        rs_pipe_->stop();
    } catch (const std::exception& e) {}

    try {
                get_camera(rs_device_);
                rs_config_.enable_stream(RS2_STREAM_DEPTH,RS2_FORMAT_Z16, 30);
                rs_pipeline_profile_ = rs_pipe_->start(rs_config_);
                auto depth_stream = rs_pipeline_profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
                intrinsics_ = depth_stream.get_intrinsics();
                realsense_depth_->width  = intrinsics_.width;
                realsense_depth_->height = intrinsics_.height;
                realsense_depth_->resize(intrinsics_.width * intrinsics_.height);
                rs2::depth_sensor sensor = rs_device_.first<rs2::depth_sensor>();
                camera_scale_ = sensor.get_depth_scale();

                std::cout <<  "Height: " << intrinsics_.height << " Width: " << intrinsics_.width << " Scale: " << camera_scale_ << std::endl;
                camera_running_ = true;
            }
        }
        catch (const rs2::error & e)
        {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            return false;
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
    return true;
}

/* Get the rs_device pointer and printout camera details
 * @return rs_device
 */
void
Realsense2Thread::get_camera(rs2::device &dev)
{
    try {
            rs_context_ = new rs2::context();
            rs2::device_list devlist = rs_context_->query_devices();
            if (devlist.size() == 0){
                 std::cerr << "No device connected, please connect a RealSense device" << std::endl;
                 return;
            }else{
                std::cout << "found devices: " << devlist.size() << std::endl;
                dev = devlist.front();
                std::string name = "Unknown Device";
                if (dev.supports(RS2_CAMERA_INFO_NAME)){
                    name = dev.get_info(RS2_CAMERA_INFO_NAME);
                } else std::cout << "name not supported" << std::endl;

                std::string sn = "########";
                if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)){
                    sn = std::string("#") + rs_device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                } else std::cout << "serial not supported" << std::endl;

                std::cout << "name: " << name << " SN: " << sn << std::endl;
            }
        }
        catch (const rs2::error & e)
        {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            return;
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return;
        }



}

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
    camera_running_ = false;
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
