
/***************************************************************************
 *  change_handler.h - Fawkes configuration change handler interface
 *
 *  Created: Mon Dec 04 18:48:54 2006
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

#include <config/change_handler.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

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
 * @fn void ConfigurationChangeHandler::config_value_changed(const char *path, bool is_default, int value)
 * Called whenever an int value has changed.
 * @param path path of value
 * @param is_default true if modified value is a default value, false otherwise
 * @param value new value
 *
 * @fn void ConfigurationChangeHandler::config_value_changed(const char *path, bool is_default, unsigned int value)
 * Called whenever an unsigned int value has changed.
 * @param path path of value
 * @param is_default true if modified value is a default value, false otherwise
 * @param value new value
 *
 * @fn void ConfigurationChangeHandler::config_value_changed(const char *path, bool is_default, float value)
 * Called whenever an float value has changed.
 * @param path path of value
 * @param is_default true if modified value is a default value, false otherwise
 * @param value new value
 *
 * @fn virtual void ConfigurationChangeHandler::config_value_changed(const char *path, bool is_default, bool value)
 * Called whenever an boolean value has changed.
 * @param path path of value
 * @param is_default true if modified value is a default value, false otherwise
 * @param value new value
 *
 * @fn void ConfigurationChangeHandler::config_value_changed(const char *path, bool is_default, const char *value)
 * Called whenever a string value has changed.
 * @param path path of value
 * @param is_default true if modified value is a default value, false otherwise
 * @param value new value
 *
 * @fn void ConfigurationChangeHandler::config_comment_changed(const char *path, bool is_default, const char *comment)
 * Called whenever a comment has changed.
 * @param path path of value
 * @param is_default true if modified comment is of a default value, false otherwise
 * @param comment new comment
 *
 * @fn void ConfigurationChangeHandler::config_value_erased(const char *path, bool is_default)
 * Called whenever a value has been erased from the config.
 * @param path path of value
 * @param is_default true if erased value was a default value, false otherwise
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
 * @return monitored path prefix
 */
const char *
ConfigurationChangeHandler::config_monitor_prefix()
{
  return __path_prefix;
}

} // end namespace fawkes
