
/***************************************************************************
 *  bblogger_plugin.h - Fawkes BlackBoard Logger Plugin
 *
 *  Created: Sat Nov 07 23:20:51 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_BBLOGGER_BBLOGGER_PLUGIN_H_
#define __PLUGINS_BBLOGGER_BBLOGGER_PLUGIN_H_

#include <core/plugin.h>

class BlackBoardLoggerPlugin : public fawkes::Plugin
{
 public:
  BlackBoardLoggerPlugin(fawkes::Configuration *config);
};

#endif
