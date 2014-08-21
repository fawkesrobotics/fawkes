
/***************************************************************************
 *  openprs_manager_inifin.h - Fawkes OpenPRSManagerAspect initializer/finalizer
 *
 *  Created: Mon Aug 18 15:25:14 2014
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __PLUGINS_OPENPRS_ASPECT_OPENPRS_MANAGER_INIFIN_H_
#define __PLUGINS_OPENPRS_ASPECT_OPENPRS_MANAGER_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <plugins/openprs/aspect/openprs_manager.h>

#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenPRSKernelManager;

class OpenPRSManagerAspectIniFin : public AspectIniFin
{
 public:
  OpenPRSManagerAspectIniFin();
  ~OpenPRSManagerAspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);

  void set_manager(LockPtr<OpenPRSKernelManager> &clips_kernel_mgr);

 private:
  LockPtr<OpenPRSKernelManager> openprs_kernel_mgr_;
};

} // end namespace fawkes

#endif
