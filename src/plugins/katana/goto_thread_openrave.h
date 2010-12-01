
/***************************************************************************
 *  goto_thread_openrave.h - Katana goto one-time thread using OpenRAVE lib
 *
 *  Created: Wed Jun 10 11:44:24 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *                  2010  Bahram Maleki-Fard
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

#ifndef __PLUGINS_KATANA_GOTO_THREAD_OPENRAVE_H_
#define __PLUGINS_KATANA_GOTO_THREAD_OPENRAVE_H_

#include "goto_thread.h"

#include <vector>

#ifdef HAVE_OPENRAVE
// openrave namespace stuff
#endif

class KatanaGotoThreadOpenRAVE : public KatanaGotoThread
{
 public:
  KatanaGotoThreadOpenRAVE(fawkes::RefPtr<CLMBase> katana, fawkes::Logger *logger,
		   unsigned int poll_interval_ms);

#ifdef HAVE_OPENRAVE
#endif //openrave check
};

#endif
