
/***************************************************************************
 *  thread_initializer.h - Fawkes thread initializer
 *
 *  Created: Thu Nov 20 00:47:12 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FAWKES_THREAD_INITIALIZER_H_
#define __FAWKES_THREAD_INITIALIZER_H_

#include <aspect/initializer.h>

class BlackBoard;
class Configuration;
class Thread;

class FawkesThreadInitializer : public AspectInitializer
{
 public:
  FawkesThreadInitializer(BlackBoard *blackboard, Configuration *config);

  virtual void init(Thread *thread);

};


#endif
