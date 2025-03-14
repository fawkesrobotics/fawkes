
/***************************************************************************
 *  pos3d_publisher_thread.h - A publisher for 3D robot positions
 *
 *  Created: Sat Apr 06 16:20:00 2024
 *  Copyright  2024  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

#ifndef _PLUGINS_POS3D_PUBLISHER_THREAD_H_
#define _PLUGINS_POS3D_PUBLISHER_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/thread.h>
#include <interfaces/Position3DInterface.h>

namespace fawkes {
namespace tf {
class TransformListener;
}
} // namespace fawkes

class Pos3dPublisherThread : public fawkes::Thread,
                             public fawkes::BlockedTimingAspect,
                             public fawkes::LoggingAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::BlackBoardAspect,
                             public fawkes::TransformAspect,
                             public fawkes::BlackBoardInterfaceObserver,
                             public fawkes::BlackBoardInterfaceListener
{
public:
	Pos3dPublisherThread();
	virtual ~Pos3dPublisherThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::Position3DInterface *pos3d_if_;
	fawkes::Position3DInterface *pos3d_if_agent_;
	std::string                  global_frame_id_;
	std::string                  cfg_pose_ifname_;
	std::string                  cfg_pose_ifname_agent_;
	unsigned int                 loop_nr_        = 0;
	unsigned int                 loop_threshold_ = 0;
};

#endif
