
/***************************************************************************
 *  plugin_tool.h - Fawkes plugin tool
 *
 *  Created: Sun Nov 26 16:44:00 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __TOOLS_PLUGIN_PLUGIN_TOOL_H_
#define __TOOLS_PLUGIN_PLUGIN_TOOL_H_

#include <netcomm/fawkes/client_handler.h>
#include <utils/system/signal.h>

namespace fawkes {
  class FawkesNetworkClient;
  class FawkesNetworkMessage;
  class ArgumentParser;
}

class PluginTool
: public fawkes::SignalHandler,
  public fawkes::FawkesNetworkClientHandler
{
 public:
  PluginTool(fawkes::ArgumentParser *argp, fawkes::FawkesNetworkClient *c);
  PluginTool(fawkes::FawkesNetworkClient *c);
  ~PluginTool();

  void handle_signal(int signum);

  void set_load_plugin(const char *plugin_name);
  void set_unload_plugin(const char *plugin_name);
  void set_watch_mode();
  void set_list_mode();

  void run();

  static void print_usage(const char *program_name);

 private:
  void load();
  void unload();
  void list_loaded();
  void watch();
  void list_avail();

  virtual void deregistered(unsigned int id) throw();
  virtual void inbound_received(fawkes::FawkesNetworkMessage *msg,
				unsigned int id) throw();
  virtual void connection_died(unsigned int id) throw();
  virtual void connection_established(unsigned int id) throw();

 private:
  typedef enum {
    M_LIST_LOADED,
    M_LIST_AVAIL,
    M_LOAD,
    M_UNLOAD,
    M_RELOAD,
    M_WATCH,
    M_UNKNOWN
  } OperationMode;

  fawkes::FawkesNetworkClient *c;
  OperationMode   opmode;
  const char     *plugin_name;
  const char     *__program_name;
  bool            quit;

  bool            list_found;
};


#endif
