
/***************************************************************************
 *  handtracker_thread.h - OpenNI hand tracker thread
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

#ifndef _PLUGINS_OPENNI_HANDTRACKER_THREAD_H_
#define _PLUGINS_OPENNI_HANDTRACKER_THREAD_H_

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
class ObjectPositionInterface;
}
class OpenNiHandTrackerThread : public fawkes::Thread,
                                public fawkes::BlockedTimingAspect,
                                public fawkes::LoggingAspect,
                                public fawkes::ConfigurableAspect,
                                public fawkes::ClockAspect,
                                public fawkes::BlackBoardAspect,
                                public fawkes::OpenNiAspect
{
public:
	OpenNiHandTrackerThread();
	virtual ~OpenNiHandTrackerThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	void hand_create(XnUserID &user, const XnPoint3D *position, XnFloat &time);
	void hand_destroy(XnUserID &user, XnFloat &time);
	void hand_update(XnUserID &user, const XnPoint3D *position, XnFloat &time);

	void gesture_recognized(const XnChar *   gesture_name,
	                        const XnPoint3D *position,
	                        const XnPoint3D *end_position);
	void gesture_progress(const XnChar *gesture_name, const XnPoint3D *position, XnFloat progress);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void update_hand(XnUserID &user, const XnPoint3D *position);

private:
	typedef std::map<XnUserID, fawkes::ObjectPositionInterface *> HandMap;

private:
	xn::HandsGenerator *  hand_gen_;
	xn::DepthGenerator *  depth_gen_;
	xn::GestureGenerator *gesture_gen_;

	XnCallbackHandle hand_cb_handle_;
	XnCallbackHandle gesture_cb_handle_;

	std::map<std::string, bool> enabled_gesture_;

	std::map<XnUserID, bool> needs_write_;
	HandMap                  hands_;

	unsigned int width_;
	unsigned int height_;
};

#endif
