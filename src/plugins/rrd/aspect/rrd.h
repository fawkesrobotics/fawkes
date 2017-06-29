
/***************************************************************************
 *  rrd.h - RRD aspect for Fawkes
 *
 *  Created: Fri Dec 17 00:19:43 2010
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

#ifndef __PLUGINS_RRD_ASPECT_RRD_H_
#define __PLUGINS_RRD_ASPECT_RRD_H_

#include <aspect/aspect.h>
#include <plugins/rrd/aspect/rrd_manager.h>
#include <plugins/rrd/aspect/rrd_descriptions.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RRDAspectIniFin;

class RRDAspect : public virtual Aspect
{
  friend RRDAspectIniFin;

 public:
  RRDAspect();
  virtual ~RRDAspect();

 protected:
  RRDManager *rrd_manager;

 private:
  void init_RRDAspect(RRDManager *rrd_manager);
};

} // end namespace fawkes

#endif
