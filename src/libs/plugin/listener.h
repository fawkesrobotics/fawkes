
/***************************************************************************
 *  listener.h - Fawkes plugin manager listener
 *
 *  Created: Thu Feb 12 10:58:49 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGIN_LISTENER_H_
#define __PLUGIN_LISTENER_H_

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class PluginManagerListener
{
 public:
  virtual ~PluginManagerListener();

  virtual void plugin_loaded(const char *plugin_name)    = 0;
  virtual void plugin_unloaded(const char *plugin_name)  = 0;
};

} // end namespace fawkes

#endif
