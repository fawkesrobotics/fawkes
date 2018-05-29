/***************************************************************************
 *  syncpoint.h - SyncPoint Aspect initializer/finalizer
 *
 *  Created: Thu Feb 19 14:39:42 2015
 *  Copyright  2015  Till Hofmann
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

#ifndef __ASPECT_INIFINS_SYNCPOINT_H_
#define __ASPECT_INIFINS_SYNCPOINT_H_

#include <aspect/inifins/inifin.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SyncPointManager;

class SyncPointAspectIniFin : public AspectIniFin
{
 public:
    SyncPointAspectIniFin(SyncPointManager *syncpoint_manager);

    virtual void init(Thread *thread);
    virtual void finalize(Thread *thread);

  private:
    SyncPointManager *__syncpoint_manager;
};

} // end namespace fawkes

#endif
