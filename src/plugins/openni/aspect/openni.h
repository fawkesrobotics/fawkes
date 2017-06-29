
/***************************************************************************
 *  openni.h - OpenNI aspect for Fawkes
 *
 *  Created: Sat Feb 26 15:34:29 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENNI_ASPECT_OPENNI_H_
#define __PLUGINS_OPENNI_ASPECT_OPENNI_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>

namespace xn {
  class Context;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenNiAspectIniFin;

class OpenNiAspect : public virtual Aspect
{
 friend OpenNiAspectIniFin;

 public:
  OpenNiAspect();
  virtual ~OpenNiAspect();

 protected:
  LockPtr<xn::Context> openni;

 private:
  void init_OpenNiAspect(LockPtr<xn::Context> openni_context);
};

} // end namespace fawkes

#endif
