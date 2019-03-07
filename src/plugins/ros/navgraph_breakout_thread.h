
/***************************************************************************
 *  navgraph_breakout_thread.h - Provide navgraph-like API through ROS
 *
 *  Created: Fri Jan 27 11:20:32 2017
 *  Copyright  2017  Tim Niemueller
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

#ifndef _ROS_NAVGRAPH_BREAKOUT_THREAD_H_
#define _ROS_NAVGRAPH_BREAKOUT_THREAD_H_

#include <actionlib/client/simple_action_client.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <fawkes_msgs/NavGraphGotoAction.h>
#include <fawkes_msgs/NavGraphGotoGoal.h>
#include <ros/ros.h>

#include <string>

namespace fawkes {
class NavigatorInterface;
}

class RosNavgraphBreakoutThread : public fawkes::Thread,
                                  public fawkes::BlockedTimingAspect,
                                  public fawkes::LoggingAspect,
                                  public fawkes::BlackBoardAspect,
                                  public fawkes::ConfigurableAspect
{
public:
	RosNavgraphBreakoutThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	typedef actionlib::SimpleActionClient<fawkes_msgs::NavGraphGotoAction> NavGraphGotoClient;

	std::string cfg_action_topic_;

	fawkes::NavigatorInterface *  pp_nav_if_;
	NavGraphGotoClient *          ac_;
	fawkes_msgs::NavGraphGotoGoal goal_;
	bool                          goal_active_;
	bool                          was_connected_;
};

#endif /* ROS_NAVIGATOR_THREAD_H__ */
