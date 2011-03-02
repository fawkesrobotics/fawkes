
/***************************************************************************
 *  tracker_thread.h - OpenNI user tracker thread
 *
 *  Created: Sun Feb 27 17:52:26 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENNI_TRACKER_THREAD_H_
#define __PLUGINS_OPENNI_TRACKER_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/openni/aspect/openni.h>

#include <XnCppWrapper.h>

#include <map>

namespace fawkes {
  class HumanSkeletonInterface;
  class HumanSkeletonProjectionInterface;
}

class OpenNiUserTrackerThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::OpenNiAspect
{
 public:
  OpenNiUserTrackerThread();
  virtual ~OpenNiUserTrackerThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  void new_user(XnUserID id);
  void lost_user(XnUserID id);
  void pose_start(XnUserID id, const char *pose_name);
  void pose_end(XnUserID id, const char *pose_name);
  void calibration_start(XnUserID id);
  void calibration_end(XnUserID id, bool success);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  typedef struct {
    fawkes::HumanSkeletonInterface            *skel_if;
    fawkes::HumanSkeletonProjectionInterface  *proj_if;
  } UserInfo;

  typedef std::map<XnUserID, UserInfo>  UserMap;

  void update_user(XnUserID id, UserInfo &user);

 private:
  xn::UserGenerator               *__user_gen;
  xn::DepthGenerator              *__depth_gen;

  XnCallbackHandle                 __user_cb_handle;
  XnCallbackHandle                 __calib_cb_handle;
  XnCallbackHandle                 __pose_cb_handle;

  char                             __calib_pose_name[32];
  bool                             __skel_need_calib_pose;

  UserMap                          __users;
};

#endif
