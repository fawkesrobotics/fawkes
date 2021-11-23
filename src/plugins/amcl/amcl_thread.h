/***************************************************************************
 *  amcl_thread.cpp - Thread to perform localization
 *
 *  Created: Wed May 16 16:03:38 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#define HAVE_ROS 1

#ifndef _PLUGINS_AMCL_AMCL_THREAD_H_
#define _PLUGINS_AMCL_AMCL_THREAD_H_

#define NEW_UNIFORM_SAMPLING 1

#include "map/map.h"
#include "pf/pf.h"
#include "pf/pf_vector.h"
#include "sensors/amcl_laser.h"
#include "sensors/amcl_odom.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/LocalizationInterface.h>
#include <interfaces/Position3DInterface.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

/// Pose hypothesis
typedef struct
{
	/// Total weight (weights sum to 1)
	double weight;
	/// Mean of pose esimate
	pf_vector_t pf_pose_mean;
	/// Covariance of pose estimate
	pf_matrix_t pf_pose_cov;
} amcl_hyp_t;

namespace fawkes {
class Mutex;
}

#ifdef HAVE_ROS
class AmclROS2Thread;
#endif

class AmclThread : public fawkes::Thread,
                   public fawkes::ClockAspect,
                   public fawkes::LoggingAspect,
                   public fawkes::ConfigurableAspect,
                   public fawkes::BlockedTimingAspect,
                   public fawkes::BlackBoardAspect,
                   public fawkes::TransformAspect,
                   public fawkes::BlackBoardInterfaceListener

{
public:
#ifdef HAVE_ROS
	AmclThread(AmclROS2Thread *ros_thread);
#else
	AmclThread();
#endif
	virtual ~AmclThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();
	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	bool               set_laser_pose();
	bool               get_odom_pose(fawkes::tf::Stamped<fawkes::tf::Pose> &odom_pose,
	                                 double                                &x,
	                                 double                                &y,
	                                 double                                &yaw,
	                                 const fawkes::Time                    *t,
	                                 const std::string                     &f);
	void               apply_initial_pose();
	static pf_vector_t uniform_pose_generator(void *arg);
	void               set_initial_pose(const std::string      &frame_id,
	                                    const fawkes::Time     &msg_time,
	                                    const fawkes::tf::Pose &pose,
	                                    const double           *covariance);
	virtual bool       bb_interface_message_received(fawkes::Interface *interface,
	                                                 fawkes::Message   *message) noexcept;

private:
	fawkes::Mutex *conf_mutex_;

	std::string cfg_map_file_;
	float       cfg_resolution_;
	float       cfg_origin_x_;
	float       cfg_origin_y_;
	float       cfg_origin_theta_;
	float       cfg_occupied_thresh_;
	float       cfg_free_thresh_;
	bool        cfg_read_init_cov_;
	bool        cfg_buffer_enable_;
	bool        cfg_buffer_debug_;
	bool        cfg_use_latest_odom_;

	std::string cfg_laser_ifname_;
	std::string cfg_pose_ifname_;

	unsigned int map_width_;
	unsigned int map_height_;
	bool         laser_pose_set_;

	fawkes::tf::Transform latest_tf_;

	amcl::odom_model_t  odom_model_type_;
	amcl::laser_model_t laser_model_type_;

	int max_beams_, min_particles_, max_particles_;

	bool   sent_first_transform_;
	bool   latest_tf_valid_;
	map_t *map_;
	pf_t  *pf_;
	int    resample_count_;

	double       save_pose_period_;
	double       transform_tolerance_;
	fawkes::Time save_pose_last_time;

	fawkes::Laser360Interface     *laser_if_;
	fawkes::Position3DInterface   *pos3d_if_;
	fawkes::LocalizationInterface *loc_if_;

	amcl_hyp_t *initial_pose_hyp_;
	bool        first_map_received_;
	bool        first_reconfigure_call_;

	// Particle filter
	double      pf_err_, pf_z_;
	bool        pf_init_;
	pf_vector_t pf_odom_pose_;
	double      laser_min_range_;
	double      laser_max_range_;

	amcl::AMCLOdom  *odom_;
	amcl::AMCLLaser *laser_;
	bool             laser_update_;
	bool             laser_buffered_;

	fawkes::Time last_cloud_pub_time;
	fawkes::Time last_laser_received_ts_;
	double       last_covariance_[36];

	float        alpha1_;
	float        alpha2_;
	float        alpha3_;
	float        alpha4_;
	float        alpha5_;
	float        z_hit_;
	float        z_short_;
	float        z_max_;
	float        z_rand_;
	float        sigma_hit_;
	float        lambda_short_;
	float        laser_likelihood_max_dist_;
	float        d_thresh_;
	float        a_thresh_;
	float        t_thresh_;
	float        alpha_slow_;
	float        alpha_fast_;
	float        init_pose_[3];
	float        init_cov_[3];
	float        angle_increment_;
	float        angle_min_;
	unsigned int angle_min_idx_;
	unsigned int angle_max_idx_;
	unsigned int angle_range_;

	unsigned int resample_interval_;

	fawkes::Time *last_move_time_;

	std::string odom_frame_id_;
	std::string base_frame_id_;
	std::string global_frame_id_;

#if NEW_UNIFORM_SAMPLING
	static std::vector<std::pair<int, int>> free_space_indices;
#endif

#ifdef HAVE_ROS
	AmclROS2Thread *rt_;
#endif
};

#endif
