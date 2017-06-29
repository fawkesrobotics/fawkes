
/***************************************************************************
 *  naoqi.h - NaoQi aspect for Fawkes
 *
 *  Created: Thu May 12 15:48:21 2011
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

#ifndef __PLUGINS_NAO_ASPECT_NAOQI_H_
#define __PLUGINS_NAO_ASPECT_NAOQI_H_

#include <aspect/aspect.h>
#include <alcore/alptr.h>
#include <alcommon/albroker.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NaoQiAspectIniFin;

class NaoQiAspect : public virtual Aspect
{
  friend NaoQiAspectIniFin;

 public:
  NaoQiAspect();
  virtual ~NaoQiAspect();

 protected:
  AL::ALPtr<AL::ALBroker>  naoqi_broker;

 private:
  void init_NaoQiAspect(AL::ALPtr<AL::ALBroker> naoqi_broker);
};

} // end namespace fawkes

#endif
