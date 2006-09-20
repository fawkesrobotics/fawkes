
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


/** Plugin interface, derive this class to create a new Fawkes plugin
 */
class Plugin {
 public:

  /** Type of the plugin
   */
  typedef enum {
    MOTION,		/**< motion plugin */
    VISION,		/**< vision plugin */
    AGENT,		/**< agent plugin */
    SKILLER		/**< skill plugin */
  } PluginType;

  /** Virtual destructor for pure virtual class
   */
  virtual ~Plugin() {}

  /** Get the type of the plugin
   * @return Returns the plugin type
   */
  virtual PluginType    getType()                                         = 0;

  /** Get the name of the plugin
   * @return Returns the name of the plugin
   */
  virtual const char *  getName()                                         = 0;

  /** Determines if the plugin can be unloaded.
   * This method tells the plugin loader if this plugin can be unloaded. Use
   * with care. No plugins but core plugins should return true. Only override
   * this if needed. The default behaviour if not overridden is to return false.
   * @return true, if the plugin cannot be unloaded, false otherwise. The default
   * implementation returns false.
   */
  virtual bool          persistent();

};

/** Plugin loader function for the shared library
 * Declare and define this function exactly like this:
 * 
 * extern "C"
 * Plugin *
 * plugin_factory()
 * {
 *  return new MightyPlugin();
 * }
 * Do not change the type or name of this function, replace MightyPlugin
 * with the name of your plugin derivative.
 */
typedef Plugin *  (* PluginFactoryFunc)  (void);

/* Plugin destructor function for the shared library.
 * Declare and define this function exactly like this:
 * extern "C"
 * void
 * plugin_destroy(Plugin *plugin)
 * {
 *   delete plugin;
 * }
 * Do not change the type or name of this function or type of arguments
 * of this function!
 */
typedef void      (* PluginDestroyFunc)  (Plugin *);


#endif
