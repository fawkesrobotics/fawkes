
/***************************************************************************
 *  plugin.h - Interface for a Fawkes plugin
 *
 *  Created: Wed Aug 23 15:19:13 2006
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_PLUGIN_H_
#define __CORE_PLUGIN_H_

#include <core/threading/thread_list.h>

namespace fawkes {

class Configuration;

class Plugin {
 public:

  Plugin(Configuration *config);
  virtual ~Plugin();

  void          set_name(const char *name);
  const char *  name() const;
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

  /** Fawkes configuration. You can use the configuration to add certain threads
   * depending on the requested feature set. Use it only in your constructor.
   */
  Configuration *config;

 private:
  char       *_name_alloc;
  const char *_name;
};

/** Plugin loader function for the shared library
 * Do not use directly, rather use the EXPORT_PLUGIN macro.
 * @relates fawkes::Plugin
 */
typedef Plugin *  (* PluginFactoryFunc)  (fawkes::Configuration *);

/** Plugin destructor function for the shared library.
 * Do not use directly, rather use the EXPORT_PLUGIN macro.
 * @param plugin plugin to destroy
 * @relates fawkes::Plugin
 */
typedef void      (* PluginDestroyFunc)  (Plugin *plugin);


/** Plugin description function for the shared library.
 * @return short string describing the plugin.
 */
typedef const char *  (* PluginDescriptionFunc) ();

/** Plugin depdendency function for the shared library.
 * @return short string with a comma separated list of plugins that this
 * plugin depends on.
 */
typedef const char *  (* PluginDependenciesFunc) ();


/** Plugin factory function for this plugin.
 * @return an instance of ExamplePlugin
 */
#define PLUGIN_FACTORY(plugin_class)			\
  extern "C"						\
  fawkes::Plugin *					\
  plugin_factory(fawkes::Configuration *config)		\
  {							\
    return new plugin_class(config);			\
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


/** Plugin description.
 * Use this macro to set a short description of the plugi
 * @param info_string a short string describing the plugin
 */
#define PLUGIN_DESCRIPTION(info_string)			\
  extern "C" const char _plugin_description[]		\
    __attribute((__section__(".fawkes_plugin")))	\
    __attribute((__used__)) = info_string;		\
							\
  extern "C"						\
  const char *						\
  plugin_description()					\
  {							\
    return _plugin_description;				\
  }



/** Set plugin dependencies.
 * @param plugin_list a string with a comma-separated list
 * of plugins that this plugin depends on.
 */
#define PLUGIN_DEPENDS(plugin_list)			\
  extern "C" const char _plugin_dependencies[]		\
    __attribute((__section__(".fawkes_plugin")))	\
    __attribute((__used__)) = info_string;		\
							\
  extern "C"						\
  const char *						\
  plugin_depends()					\
  {							\
    return _plugin_dependencies;			\
  }



/** Export plugin.
 * This will create appropriate plugin factory and destroy functions.
 */
#define EXPORT_PLUGIN(plugin_class) \
  PLUGIN_FACTORY(plugin_class)      \
  				    \
  PLUGIN_DESTROY(plugin_class)


} // end namespace fawkes

#endif
