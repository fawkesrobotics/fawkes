
/***************************************************************************
 *  usertracker_thread.h - OpenNI user tracker thread
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

#ifndef _PLUGINS_OPENNI_USERTRACKER_THREAD_H_
#define _PLUGINS_OPENNI_USERTRACKER_THREAD_H_

#include "utils/version.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <plugins/openni/aspect/openni.h>

#include <XnCppWrapper.h>
#include <map>

namespace fawkes {
class HumanSkeletonInterface;
class HumanSkeletonProjectionInterface;
} // namespace fawkes
namespace firevision {
class SharedMemoryImageBuffer;
}

class OpenNiUserTrackerThread : public fawkes::Thread,
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
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	/** Per user info struct. */
	typedef struct
	{
		bool                                      valid;   /**< true if valid */
		fawkes::HumanSkeletonInterface *          skel_if; /**< Skeleton interface */
		fawkes::HumanSkeletonProjectionInterface *proj_if; /**< Projection interface. */
	} UserInfo;

	typedef std::map<XnUserID, UserInfo> UserMap;

	void update_user(XnUserID id, UserInfo &user);
	void update_com(XnUserID id, UserInfo &user);

private:
	xn::UserGenerator * user_gen_;
	xn::DepthGenerator *depth_gen_;

	xn::SceneMetaData *     scene_md_;
	xn::SkeletonCapability *skelcap_;

	XnCallbackHandle user_cb_handle_;
#if XN_VERSION_GE(1, 3, 2, 0)
	XnCallbackHandle pose_start_cb_handle_;
	XnCallbackHandle pose_end_cb_handle_;
	XnCallbackHandle calib_start_cb_handle_;
	XnCallbackHandle calib_complete_cb_handle_;
#else
	XnCallbackHandle pose_cb_handle_;
	XnCallbackHandle calib_cb_handle_;
#endif

	char calib_pose_name_[32];
	bool skel_need_calib_pose_;

	UserMap users_;

	firevision::SharedMemoryImageBuffer *label_buf_;
	size_t                               label_bufsize_;
};

#endif
