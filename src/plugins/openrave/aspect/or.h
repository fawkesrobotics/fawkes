
/***************************************************************************
 *  or.h - OpenRAVE aspect for Fawkes
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#ifndef __PLUGINS_OPENRAVE_ASPECT_OR_H_
#define __PLUGINS_OPENRAVE_ASPECT_OR_H_

#include <aspect/aspect.h>
#include <plugins/openrave/aspect/or_connector.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRAVEAspect : public virtual Aspect
{
  friend class OpenRAVEAspectIniFin;

 public:
  OpenRAVEAspect();
  virtual ~OpenRAVEAspect();

 protected:
  OpenRAVEConnector *openrave;

 private:
  void init_OpenRAVEAspect(OpenRAVEConnector *openrave);
};

} // end namespace fawkes

#endif
