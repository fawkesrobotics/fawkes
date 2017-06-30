
/***************************************************************************
 *  clips_tf_thread.h - Transforms feature for CLIPS
 *
 *  Created: Sat Apr 11 17:28:18 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_TF_CLIPS_TF_THREAD_H_
#define __PLUGINS_CLIPS_TF_CLIPS_TF_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips_feature.h>
#include <aspect/tf.h>

#include <clipsmm.h>

class ClipsTFThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::TransformAspect,
  public fawkes::CLIPSFeature,
  public fawkes::CLIPSFeatureAspect
{
 public:
  ClipsTFThread();
  virtual ~ClipsTFThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for CLIPSFeature
  virtual void clips_context_init(const std::string &env_name,
				  fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  bool          validate_time(const CLIPS::Values &time);
  fawkes::Time  convert_time(const CLIPS::Values &time);
  bool          validate_point(const CLIPS::Values &point);
  bool          validate_vector3(const CLIPS::Values &vector3);
  bool          validate_quat(const CLIPS::Values &quat);

  CLIPS::Values clips_tf_quat_from_yaw(double yaw);
  double        clips_tf_yaw_from_quat(CLIPS::Values quat);

  CLIPS::Value  clips_tf_frame_exists(std::string frame_id);
  CLIPS::Value  clips_tf_can_transform(std::string target_frame, std::string source_frame,
				       CLIPS::Values time);
  CLIPS::Values clips_tf_transform_point(std::string target_frame, std::string source_frame,
					 CLIPS::Values time, CLIPS::Values point);
  CLIPS::Values clips_tf_transform_vector(std::string target_frame, std::string source_frame,
					  CLIPS::Values time, CLIPS::Values vector);
  CLIPS::Values clips_tf_transform_quaternion(std::string target_frame,
					      std::string source_frame,
					      CLIPS::Values time, CLIPS::Values quat);

  CLIPS::Values clips_tf_transform_pose(std::string target_frame, std::string source_frame,
					CLIPS::Values time,
					CLIPS::Values translation,
					CLIPS::Values rotation_quat);

 private:
  std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;
  bool debug_;

};

#endif
