
/***************************************************************************
 *  sensaqt_thread.cpp - Katana sensor acqusition thread
 *
 *  Created: Fri Jun 12 15:08:56 2009
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

#include "sensacq_thread.h"
#include "controller.h"

#include <cstdlib>

using namespace fawkes;

/** @class KatanaSensorAcquisitionThread "sensacq_thread.h"
 * Katana sensor acquisition thread.
 * This thread runs continuously and acquires data from the sensor. Since the
 * operation is blocking and may take several miliseconds it is done concurrently
 * to the main loop at specified intervals.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param katana katana controller base class
 * @param logger logger
 */
KatanaSensorAcquisitionThread::KatanaSensorAcquisitionThread(fawkes::RefPtr<fawkes::KatanaController> katana,
							     fawkes::Logger *logger)
  : Thread("KatanaSensorAcqusitionThread", Thread::OPMODE_WAITFORWAKEUP)
{
  __katana  = katana;
  __logger  = logger;
  __enabled = false;
}


/** Set whether data acquisition is enabled or not.
 * In general the thread should only be woken up if sensor data can be acquired.
 * But for safety data acqusition can also be turned off to be safe against
 * spurious wakeups. Additionally, this method will acquire the loop mutex,
 * thereby assuring that a possibly running loop has finished.
 * @param enabled true to enable sensor data acquisition, false to disable.
 */
void
KatanaSensorAcquisitionThread::set_enabled(bool enabled)
{
  loop_mutex->lock();
  __enabled = enabled;
  loop_mutex->unlock();
}


void
KatanaSensorAcquisitionThread::loop()
{
  if (__enabled) {
    try {
      __katana->read_sensor_data();
    } catch (Exception &e) {
      __logger->log_warn(name(), e.what());
    }
  }
}
