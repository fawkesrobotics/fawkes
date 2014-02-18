
/***************************************************************************
 *  fam_thread.h - File Alteration Monitor Thread
 *
 *  Created: Sat Jan 24 12:07:58 2009
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

#ifndef __UTILS_SYSTEM_FAM_THREAD_H_
#define __UTILS_SYSTEM_FAM_THREAD_H_

#include <core/utils/refptr.h>
#include <utils/system/fam.h>
#include <core/threading/thread.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FamThread : public Thread
{
 public:
  FamThread(RefPtr<FileAlterationMonitor> fam = RefPtr<FileAlterationMonitor>());

  RefPtr<FileAlterationMonitor> get_fam();

  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  RefPtr<FileAlterationMonitor> __fam;
};


} // end of namespace fawkes


#endif
