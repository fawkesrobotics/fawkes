
/***************************************************************************
 *  dp_thread.h - DirectedPerception pan/tilt unit act thread
 *
 *  Created: Sun Jun 21 17:26:33 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PANTILT_DIRPERC_DP_THREAD_H_
#define __PLUGINS_PANTILT_DIRPERC_DP_THREAD_H_

#include "../act_thread.h"

#include <blackboard/interface_listener.h>

#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <string>
#include <memory>

namespace fawkes {
  class PanTiltInterface;
  class JointInterface;
}

class DirectedPerceptionPTU;

class PanTiltDirectedPerceptionThread
: public PanTiltActThread,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  PanTiltDirectedPerceptionThread(std::string &pantilt_cfg_prefix,
				  std::string &ptu_cfg_prefix,
				  std::string &ptu_name);

  virtual void init();
  virtual void finalize();
  virtual void loop();

  // For BlackBoardInterfaceListener
  virtual bool bb_interface_message_received(fawkes::Interface *interface,
					     fawkes::Message *message) throw();

  void update_sensor_values();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::PanTiltInterface *__pantilt_if;
  fawkes::JointInterface *__panjoint_if;
  fawkes::JointInterface *__tiltjoint_if;

  fawkes::RefPtr<DirectedPerceptionPTU> __ptu;

  std::string  __pantilt_cfg_prefix;
  std::string  __ptu_cfg_prefix;
  std::string  __ptu_name;
  std::string  __cfg_device;
  unsigned int __cfg_read_timeout_ms;


  class WorkerThread : public fawkes::Thread
  {
  public:
    WorkerThread(std::string ptu_name, fawkes::Logger *logger,
		 fawkes::RefPtr<DirectedPerceptionPTU> ptu);

    ~WorkerThread();
    void goto_pantilt(float pan, float tilt);
    void get_pantilt(float &pan, float &tilt);
    bool is_final();
    void stop_motion();
    bool has_fresh_data();
    void reset();

    virtual void loop();

  private:
    void exec_goto_pantilt(float pan, float tilt);

  private:
    fawkes::RefPtr<DirectedPerceptionPTU>  __ptu;
    fawkes::Logger                        *__logger;

    float         __pan_min;
    float         __pan_max;
    float         __tilt_min;
    float         __tilt_max;

    fawkes::Mutex *__move_mutex;
    bool  __move_pending;
    float __target_pan;
    float __target_tilt;

    float __cur_pan;
    float __cur_tilt;

    bool  __reset_pending;
    bool  __fresh_data;
  };

  WorkerThread *__wt;
};

#endif
