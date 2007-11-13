
/***************************************************************************
 *  change_handler.h - Fawkes configuration change handler interface
 *
 *  Created: Mon Dec 04 18:48:54 2006
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <config/change_handler.h>

#include <cstring>
#include <cstdlib>

/** @class ConfigurationChangeHandler <config/change_handler.h>
 * Interface for configuration change handling.
 * One of the major flaws in the old software was that for each
 * configuration change the software had to be restarted. To avoid this
 * change handlers are introduced. Change handlers are called if a value
 * for the given component changes so that appropriate adjustement of the
 * behavior or a proper re-initialisation of a specific component can
 * be conducted.
 * @author Tim Niemueller 
 *
 * @fn void ConfigurationChangeHandler::config_tag_changed(const char *new_tag)
 * Called whenever the tag has changed.
 * This function can be used to detect when data from another tag has been
 * loaded.
 * @param new_tag new tag
 *
 * @fn void ConfigurationChangeHandler::config_value_changed(const char *path, int value)
 * Called whenever an int value has changed.
 * @param path path of value
 * @param value new value
 *
 * @fn void ConfigurationChangeHandler::config_value_changed(const char *path, unsigned int value)
 * Called whenever an unsigned int value has changed.
 * @param path path of value
 * @param value new value
 *
 * @fn void ConfigurationChangeHandler::config_value_changed(const char *path, float value)
 * Called whenever an float value has changed.
 * @param path path of value
 * @param value new value
 *
 * @fn virtual void ConfigurationChangeHandler::config_value_changed(const char *path, bool value)
 * Called whenever an boolean value has changed.
 * @param path path of value
 * @param value new value
 *
 * @fn void ConfigurationChangeHandler::config_value_changed(const char *path, const char *value)
 * Called whenever a string value has changed.
 * @param path path of value
 * @param value new value
 *
 * @fn void ConfigurationChangeHandler::config_value_erased(const char *path)
 * Called whenever a value has been erased from the config.
 * @param path path of value
 */


/** Constructor.
 * @param path_prefix Path prefix to monitor. Use the empty string ("") to
 * monitor all changes.
 */
ConfigurationChangeHandler::ConfigurationChangeHandler(const char *path_prefix)
{
  __path_prefix = strdup(path_prefix);
}



/** Destructor. */
ConfigurationChangeHandler::~ConfigurationChangeHandler()
{
  free(__path_prefix);
}


/** Which path prefix shall be monitored.
 * Implement this method to return the name of the component whose values you
 * want to monitor. If NULL or the empty string is returned all components
 * will be monitored.
 */
const char *
ConfigurationChangeHandler::config_monitor_prefix()
{
  return __path_prefix;
}
