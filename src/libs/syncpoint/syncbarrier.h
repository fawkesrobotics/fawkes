/***************************************************************************
 *  syncbarrier.h - Fawkes SyncBarrier
 *
 *  Created: Fri Jan 16 14:02:42 2015
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

#ifndef __SYNCPOINT_SYNCBARRIER_H_
#define __SYNCPOINT_SYNCBARRIER_H_

#include <syncpoint/syncpoint.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


class SyncBarrier : public SyncPoint
{
  public:
    SyncBarrier(std::string identifier);

    /** wait for the barrier */
    virtual void wait(const std::string & component);

};

} // end namespace fawkes

#endif
