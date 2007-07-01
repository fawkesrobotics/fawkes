
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
    WORLDMODEL,		/**< world model plugin */
    MOTION,		/**< motion plugin */
    VISION,		/**< vision plugin */
    AGENT,		/**< agent plugin */
    SKILLER		/**< skill plugin */
  } PluginType;

  Plugin(PluginType plugin_type, const char *plugin_name);
  virtual ~Plugin();

  virtual PluginType    type() const;
  virtual const char *  name() const;

  ThreadList &  threads();

  virtual bool          persistent();

 protected:
  /** Thread list member. Initialise this list with the threads that this
   * plugin will use. These threads must exist for the whole life time of
   * the thread. Use sleeping threads if you need to turn on and off threads
   * dynamically. You may not add threads later to the list, as the list
   * is shortly after the constructor sealed.
   * @see ThreadList
   */
  ThreadList thread_list;

 private:
  char       *_name;
  PluginType  _type;

};

typedef Plugin *  (* PluginFactoryFunc)  (void);
typedef void      (* PluginDestroyFunc)  (Plugin *);

/** Plugin factory function for this plugin.
 * @return an instance of ExamplePlugin
 */
#define PLUGIN_FACTORY(plugin_class)			\
  extern "C"						\
  Plugin *						\
  plugin_factory()					\
  {							\
    return new plugin_class();				\
  }


/** Plugin destruction function for this plugin.
 * @param plugin The plugin that is to be destroyed. Do not use this plugin
 *        afterwards
 */
#define PLUGIN_DESTROY(plugin_class)			\
  extern "C"						\
  void							\
  plugin_destroy(plugin_class *plugin)			\
  {							\
    delete plugin;					\
  }

/** Export plugin.
 * This will create appropriate plugin factory and destroy functions.
 */
#define EXPORT_PLUGIN(plugin_class) \
  PLUGIN_FACTORY(plugin_class)      \
  				    \
  PLUGIN_DESTROY(plugin_class)

#endif
