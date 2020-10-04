
/***************************************************************************
 *  realsense2_thread.h - realsense2
 *
 *  Plugin created: Wed May 22 10:09:22 2019

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

#ifndef _PLUGINS_REALSENSE2THREAD_H_
#define _PLUGINS_REALSENSE2THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/pointcloud.h>
#include <core/threading/thread.h>
#include <librealsense2/rsutil.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <string>
#include <thread>

namespace fawkes {
class SwitchInterface;
class CameraControlInterface;
} // namespace fawkes

class Realsense2Thread : public fawkes::Thread,
                         public fawkes::BlockedTimingAspect,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::PointCloudAspect,
                         public fawkes::ClockAspect
{
public:
	Realsense2Thread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

private:
	bool start_camera();
	bool get_camera(rs2::device &dev);
	void enable_depth_stream();
	void disable_depth_stream();
	void stop_camera();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

protected:
	bool        read_switch();
	std::string read_camera_control();

private:
	fawkes::SwitchInterface *       switch_if_;
	bool                            cfg_use_switch_;
	fawkes::CameraControlInterface *camera_if_;

	typedef pcl::PointXYZ              PointType;
	typedef pcl::PointCloud<PointType> Cloud;

	typedef Cloud::Ptr      CloudPtr;
	typedef Cloud::ConstPtr CloudConstPtr;

	fawkes::RefPtr<Cloud> realsense_depth_refptr_;
	CloudPtr              realsense_depth_;

	rs2::pipeline *rs_pipe_;
	rs2::context * rs_context_;
	rs2::device    rs_device_;
	rs2::frameset  rs_data_;
	rs2_intrinsics intrinsics_;
	rs2_intrinsics rgb_intrinsics_;

	float       camera_scale_;
	std::string frame_id_;
	std::string pcl_id_;
	std::string switch_if_name_;
	std::string rgb_path_;
	std::string image_name_;
	std::string camera_if_name_;
	uint        frame_rate_;
	float       laser_power_;
	bool        camera_running_ = false;
	bool        enable_camera_  = true;
	bool        depth_enabled_  = false;
	uint        restart_after_num_errors_;
	uint        error_counter_ = 0;
};

#endif
