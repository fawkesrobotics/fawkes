
/***************************************************************************
 *  act_thread.h - Colli Act Thread
 *
 *  Created: Thu Oct 17 16:58:00 2013
 *  Copyright  2013-2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_ACT_THREAD_H_
#define __PLUGINS_COLLI_ACT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/tf.h>

#include <interfaces/NavigatorInterface.h>

#include <string>

namespace ros {
  class Subscriber;
}

namespace fawkes
{
  class NavigatorInterface;
}

class ColliThread;

class ColliActThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::TransformAspect
{
 public:
  ColliActThread(ColliThread* colli_thread);
  virtual ~ColliActThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:

  ColliThread*   thread_colli_;

  fawkes::NavigatorInterface* if_navi_;

  ros::Subscriber* sub_;

  std::string cfg_iface_navi_;

  std::string cfg_frame_odom_;

  // default parameters, read from config
  float cfg_security_distance_;
  float cfg_max_velocity_;
  float cfg_max_rotation_;
  float cfg_escaping_enabled_;
  bool  cfg_stop_at_target_;
  fawkes::NavigatorInterface::OrientationMode cfg_orient_mode_;
  fawkes::NavigatorInterface::DriveMode       cfg_drive_mode_;

  // methods mainly transfered from libmonaco
  bool colli_final();
  void colli_stop();
  void colli_relgoto(float x, float y, float ori);
  void colli_goto(float x, float y, float ori);
};

#endif

