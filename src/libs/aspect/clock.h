
/***************************************************************************
 *  clock.h - Clock aspect for Fawkes
 *
 *  Created: Tue June 12 18:36:09 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __ASPECT_CLOCK_H_
#define __ASPECT_CLOCK_H_

class Clock;

class ClockAspect
{
 public:
    virtual ~ClockAspect();

    void initClockAspect(Clock* clock);

 protected:
    Clock* clock;
};

#endif /*__ASPECT_CLOCK_H_ */
