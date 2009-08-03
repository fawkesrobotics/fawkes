
/***************************************************************************
 *  fam_thread.cpp - File Alteration Monitor Thread
 *
 *  Created: Sat Jan 24 12:30:10 2009
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

#include <utils/system/fam_thread.h>

#include <unistd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FamThread <utils/system/fam_thread.h>
 * FileAlterationMonitor thread wrapper.
 * This thread wraps a FileAlterationMonitor and runs it continuously possibly
 * causing FAM events in this thread context. This thread is useful if you have
 * no good place to call FileAlterationMonitor::process_events() in your
 * part.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param fam optional RefPtr to FileAlterationMonitor. If none is given
 * one is instantiated by the FamThread instance.
 */
FamThread::FamThread(RefPtr<FileAlterationMonitor> fam)
  : Thread("FileAlterationMonitorThread", Thread::OPMODE_CONTINUOUS)
{
  __fam = fam;
  if (! __fam) {
    __fam = RefPtr<FileAlterationMonitor>(new FileAlterationMonitor());
  }
}

/** Get FileAlterationMonitor.
 * @return shared pointer to FileAlterationMonitor instance.
 */
RefPtr<FileAlterationMonitor>
FamThread::get_fam()
{
  return __fam;
}


void
FamThread::loop()
{
  __fam->process_events(-1);
  usleep(0);
}

} // end of namespace fawkes
