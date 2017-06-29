
/***************************************************************************
 *  openprs_manager.h - OpenPRS manager aspect for Fawkes
 *
 *  Created: Mon Aug 18 15:24:17 2014
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENPRS_ASPECT_OPENPRS_MANAGER_H_
#define __PLUGINS_OPENPRS_ASPECT_OPENPRS_MANAGER_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>
#include <plugins/openprs/aspect/openprs_kernel_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenPRSManagerAspectIniFin;

class OpenPRSManagerAspect : public virtual Aspect
{
  friend OpenPRSManagerAspectIniFin;

 public:
  OpenPRSManagerAspect();
  virtual ~OpenPRSManagerAspect();

 protected:
  LockPtr<OpenPRSKernelManager> openprs_kernel_mgr;

};

} // end namespace fawkes

#endif
