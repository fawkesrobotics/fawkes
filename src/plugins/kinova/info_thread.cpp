
/***************************************************************************
 *  info_thread.cpp - Kinova plugin Jaco information thread
 *
 *  Created: Thu Jun 13 19:14:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#include "info_thread.h"
#include "types.h"
#include "arm.h"

#include <interfaces/JacoInterface.h>

using namespace fawkes;

/** @class KinovaInfoThread "info_thread.h"
 * Jaco Arm information thread.
 * This thread basically provides all informationen to interfaces.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaInfoThread::KinovaInfoThread()
  : Thread("KinovaInfoThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
  __arms = NULL;
}


/** Destructor. */
KinovaInfoThread::~KinovaInfoThread()
{
}

void
KinovaInfoThread::init()
{
  __arms = new std::list<jaco_arm_t*>();
}

void
KinovaInfoThread::finalize()
{
  delete(__arms);
}

void
KinovaInfoThread::register_arm(jaco_arm_t *arm) {
  __arms->push_back(arm);
}

void
KinovaInfoThread::unregister_arms() {
  for(__arm=__arms->begin(); __arm!=__arms->end(); ++__arm)
    (*__arm) = NULL;
}

void
KinovaInfoThread::loop()
{
  if( __arms->size() == 0 )
    return;

  for(__arm=__arms->begin(); __arm!=__arms->end(); ++__arm) {

    (*__arm)->iface->set_connected(true);

    try {
      (*__arm)->arm->get_coords(__cpos);
      (*__arm)->iface->set_x(__cpos.at(0));
      (*__arm)->iface->set_y(__cpos.at(1));
      (*__arm)->iface->set_z(__cpos.at(2));
      (*__arm)->iface->set_euler1(__cpos.at(3));
      (*__arm)->iface->set_euler2(__cpos.at(4));
      (*__arm)->iface->set_euler3(__cpos.at(5));

      (*__arm)->arm->get_fingers(__cpos);
      (*__arm)->iface->set_finger1(__cpos.at(0));
      (*__arm)->iface->set_finger2(__cpos.at(1));
      (*__arm)->iface->set_finger3(__cpos.at(2));

      (*__arm)->arm->get_joints(__apos);
      for(unsigned int i=0; i<__apos.size(); i++) {
        (*__arm)->iface->set_joints(i, __apos.at(i));
      }

    } catch(fawkes::Exception &e) {
      logger->log_warn(name(), "Could not get position and joint values. Er: %s", e.what());
    }
  }
}
