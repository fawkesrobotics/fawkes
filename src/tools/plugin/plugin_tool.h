
/***************************************************************************
 *  plugin_tool.h - Fawkes plugin tool
 *
 *  Created: Sun Nov 26 16:44:00 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __TOOLS_PLUGIN_PLUGIN_TOOL_H_
#define __TOOLS_PLUGIN_PLUGIN_TOOL_H_

#include <utils/system/signal.h>

class FawkesNetworkClient;
class ArgumentParser;

class PluginTool : public SignalHandler
{
 public:
  PluginTool(ArgumentParser *argp, FawkesNetworkClient *c);
  ~PluginTool();

  void handle_signal(int signum);

  void load();
  void unload();
  void list();
  void watch();
  void run();

 private:
  typedef enum {
    M_LIST,
    M_LOAD,
    M_UNLOAD,
    M_WATCH
  } OperationMode;

  FawkesNetworkClient *c;
  OperationMode   opmode;
  ArgumentParser *argp;
  const char     *plugin_name;
  bool            quit;
};


#endif
