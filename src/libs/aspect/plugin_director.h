
/***************************************************************************
 *  plugin_director.h - Plugin director aspect for Fawkes
 *
 *  Created: Thu Feb 12 12:00:48 2009
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

#ifndef __ASPECT_PLUGIN_DIRECTOR_H_
#define __ASPECT_PLUGIN_DIRECTOR_H_

#include <aspect/aspect.h>
#include <plugin/manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class PluginDirectorAspect : public virtual Aspect
{
 public:
  PluginDirectorAspect();
  virtual ~PluginDirectorAspect();

  void init_PluginDirectorAspect(PluginManager *manager);

 protected:
  PluginManager *plugin_manager;
};

} // end namespace fawkes

#endif
