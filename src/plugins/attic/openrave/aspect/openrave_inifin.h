
/***************************************************************************
 *  openrave_inifin.h - Fawkes OpenRaveAspect initializer/finalizer
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

#ifndef _PLUGINS_OPENRAVE_ASPECT_OPENRAVE_INIFIN_H_
#define _PLUGINS_OPENRAVE_ASPECT_OPENRAVE_INIFIN_H_

#include <aspect/inifins/inifin.h>

#include <vector>

namespace fawkes {

class OpenRaveConnector;

class OpenRaveAspectIniFin : public AspectIniFin
{
public:
	OpenRaveAspectIniFin(OpenRaveConnector *openrave);

	virtual void init(Thread *thread);
	virtual void finalize(Thread *thread);

private:
	OpenRaveConnector *openrave_;
};

} // end namespace fawkes

#endif
