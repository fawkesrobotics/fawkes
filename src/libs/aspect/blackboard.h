
/***************************************************************************
 *  blackboard.h - BlackBoard aspect for Fawkes
 *
 *  Created: Thu Jan 11 16:28:58 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_BLACKBOARD_H_
#define __ASPECT_BLACKBOARD_H_

#include <aspect/aspect.h>
#include <blackboard/blackboard.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BlackBoardAspectIniFin;

class BlackBoardAspect : public virtual Aspect
{
  friend BlackBoardAspectIniFin;
 public:
  BlackBoardAspect(const char *owner = NULL);
  virtual ~BlackBoardAspect();

  void init_BlackBoardAspect(BlackBoard *bb);

 protected:
  BlackBoard *blackboard;

 private:
  const char *blackboard_owner_name_;
};

} // end namespace fawkes

#endif
