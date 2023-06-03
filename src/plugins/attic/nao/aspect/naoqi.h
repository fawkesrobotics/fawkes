
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

#ifndef _PLUGINS_NAO_ASPECT_NAOQI_H_
#define _PLUGINS_NAO_ASPECT_NAOQI_H_

#include <alcommon/albroker.h>
#include <alcore/alptr.h>
#include <aspect/aspect.h>

namespace fawkes {

class NaoQiAspectIniFin;

class NaoQiAspect : public virtual Aspect
{
	friend NaoQiAspectIniFin;

public:
	NaoQiAspect();
	virtual ~NaoQiAspect();

protected:
	AL::ALPtr<AL::ALBroker> naoqi_broker;

private:
	void init_NaoQiAspect(AL::ALPtr<AL::ALBroker> naoqi_broker);
};

} // end namespace fawkes

#endif
