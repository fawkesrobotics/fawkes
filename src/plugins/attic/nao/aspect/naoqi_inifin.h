
/***************************************************************************
 *  naoqi_inifin.h - Fawkes NaoQiAspect initializer/finalizer
 *
 *  Created: Thu May 12 15:53:07 2011
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

#ifndef _PLUGINS_NAO_ASPECT_NAOQI_INIFIN_H_
#define _PLUGINS_NAO_ASPECT_NAOQI_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <plugins/nao/aspect/naoqi.h>

namespace fawkes {

class NaoQiAspectIniFin : public AspectIniFin
{
public:
	NaoQiAspectIniFin();

	virtual void init(Thread *thread);
	virtual void finalize(Thread *thread);

	void set_naoqi_broker(AL::ALPtr<AL::ALBroker> naoqi_broker);

private:
	AL::ALPtr<AL::ALBroker> naoqi_broker_;
};

} // end namespace fawkes

#endif
