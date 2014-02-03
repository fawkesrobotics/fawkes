/***************************************************************************
 *  syncpoint_manager.h - SyncPointManager Aspect
 *
 *  Created: Thu Jan 09 12:22:10 2014
 *  Copyright  2014  Till Hofmann
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

#ifndef __ASPECT_SYNCPOINT_MANAGER_H_
#define __ASPECT_SYNCPOINT_MANAGER_H_

#include <aspect/aspect.h>
#include <syncpoint/syncpoint_manager.h>

#include <set>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SyncPointManagerAspect : public virtual Aspect
{
  public:
  SyncPointManagerAspect();
  virtual ~SyncPointManagerAspect();

  void init_SyncPointManagerAspect(SyncPointManager *syncpoint_manager);

  protected:
  SyncPointManager *syncpoint_manager;
};

} // end namespace fawkes

#endif
