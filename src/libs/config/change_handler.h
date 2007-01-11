
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

#ifndef __CONFIG_CHANGE_HANDLER_H_
#define __CONFIG_CHANGE_HANDLER_H_

#include <string>

/** @class ConfigurationChangeHandler config/change_handler.h
 * Interface for configuration change handling.
 * One of the major flaws in the old software was that for each
 * configuration change the software had to be restarted. To avoid this
 * change handlers are introduced. Change handlers are called if a value
 * for the given component changes so that appropriate adjustement of the
 * behavior or a proper re-initialisation of a specific component can
 * be conducted.
 */

class ConfigurationChangeHandler
{
 public:
  /** Virtual empty destructor. */
  virtual ~ConfigurationChangeHandler() {}

  /** Called whenever the tag has changed.
   * This function can be used to detect when data from another tag has been
   * loaded.
   * @param new_tag new tag
   */
  virtual void configTagChanged(const char *new_tag)                  = 0;

  /** Called whenever an int value has changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  int value)                               = 0;

  /** Called whenever an unsigned int value has changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  unsigned int value)                      = 0;

  /** Called whenever an float value has changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  float value)                             = 0;

  /** Called whenever an boolean value has changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  bool value)                              = 0;

  /** Called whenever a string value has changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  std::string value)                       = 0;


  /** Called whenever a value has been erased from the config.
   * @param component component of value
   * @param path path of value
   */
  virtual void configValueErased(const char *component, const char *path)  = 0;

  /** Which component shall be monitored.
   * Implement this method to return the name of the component whose values you
   * want to monitor. If NULL or the empty string is returned all components
   * will be monitored.
   */
  virtual const char *  configMonitorComponent()                           = 0;

};

#endif
