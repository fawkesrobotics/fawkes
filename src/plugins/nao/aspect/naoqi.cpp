
/***************************************************************************
 *  naoqi.cpp - NaoQi aspect for Fawkes
 *
 *  Created: Thu May 12 15:50:44 2011
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

#include <plugins/nao/aspect/naoqi.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NaoQiAspect <plugins/nao/aspect/naoqi.h>
 * Thread aspect to get access to NaoQi broker.
 * Give this aspect to your thread to use features provided by NaoQi.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var AL::ALPtr<AL::ALBroker> NaoQiAspect::naoqi_broker
 * NaoQi broker.
 * Use this broker to access features provided by NaoQi and its modules.
 */

/** Constructor. */
NaoQiAspect::NaoQiAspect()
{
  add_aspect("NaoQiAspect");
}


/** Virtual empty destructor. */
NaoQiAspect::~NaoQiAspect()
{
}


/** Init OpenNI aspect.
 * This set the OpenNI context.
 * It is guaranteed that this is called for an OpenNI Thread before start
 * is called (when running regularly inside Fawkes).
 * @param openni_context OpenNI context to use
 */
void
NaoQiAspect::init_NaoQiAspect(AL::ALPtr<AL::ALBroker> naoqi_broker)
{
  this->naoqi_broker = naoqi_broker;
}

} // end namespace fawkes
