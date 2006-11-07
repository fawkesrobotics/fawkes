
/***************************************************************************
 *  plugin.h - Interface for a Fawkes plugin
 *
 *  Generated: Wed Aug 23 15:19:13 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __CORE_PLUGIN_H_
#define __CORE_PLUGIN_H_

#include <core/threading/thread_list.h>

class Plugin {
 public:

  /** Type of the plugin */
  typedef enum {
    BLACKBOARD,		/**< BlackBoard plugin */
    MOTION,		/**< motion plugin */
    VISION,		/**< vision plugin */
    AGENT,		/**< agent plugin */
    SKILLER		/**< skill plugin */
  } PluginType;

  virtual ~Plugin();

  virtual PluginType    type() const                                   = 0;
  virtual const char *  name() const                                   = 0;
  virtual ThreadList &  threads()                                      = 0;

  virtual bool          persistent();

};

typedef Plugin *  (* PluginFactoryFunc)  (void);
typedef void      (* PluginDestroyFunc)  (Plugin *);

#endif
