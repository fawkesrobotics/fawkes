
/***************************************************************************
 *  change_handler.h - Fawkes configuration change handler interface
 *
 *  Created: Mon Dec 04 18:48:54 2006
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

#ifndef __CONFIG_CHANGE_HANDLER_H_
#define __CONFIG_CHANGE_HANDLER_H_

#include <config/config.h>

namespace fawkes {

class ConfigurationChangeHandler
{
 public:
  ConfigurationChangeHandler(const char *path_prefix);
  virtual ~ConfigurationChangeHandler();

  virtual void config_tag_changed(const char *new_tag)                       = 0;
  virtual void config_value_changed(const Configuration::ValueIterator *v) = 0;
  virtual void config_comment_changed(const Configuration::ValueIterator *v) = 0;
  virtual void config_value_erased(const char *path) = 0;

  const char *  config_monitor_prefix();

 private:
  char *__path_prefix;

};

} // end namespace fawkes

#endif
