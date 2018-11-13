
/***************************************************************************
 *  rrd_inifin.h - Fawkes RRDAspect initializer/finalizer
 *
 *  Created: Fri Dec 17 00:25:03 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_RRD_ASPECT_RRD_INIFIN_H_
#define _PLUGINS_RRD_ASPECT_RRD_INIFIN_H_

#include <aspect/inifins/inifin.h>

#include <vector>

namespace fawkes {

class RRDManager;

class RRDAspectIniFin : public AspectIniFin
{
 public:
  RRDAspectIniFin(RRDManager *rrd_manager);

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);

 private:
  RRDManager *rrd_manager_;
};

} // end namespace fawkes

#endif
