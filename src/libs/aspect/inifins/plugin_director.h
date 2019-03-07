
/***************************************************************************
 *  plugin_director.h - Fawkes PluginDirector Aspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:40:17 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef _ASPECT_INIFINS_PLUGIN_DIRECTOR_H_
#define _ASPECT_INIFINS_PLUGIN_DIRECTOR_H_

#include <aspect/inifins/inifin.h>

namespace fawkes {

class PluginManager;

class PluginDirectorAspectIniFin : public AspectIniFin
{
public:
	PluginDirectorAspectIniFin(PluginManager *manager);

	virtual void init(Thread *thread);
	virtual void finalize(Thread *thread);

private:
	PluginManager *manager_;
};

} // end namespace fawkes

#endif
