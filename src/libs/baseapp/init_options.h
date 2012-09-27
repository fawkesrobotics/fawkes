
/***************************************************************************
 *  init_options.h - Fawkes run-time initialization options
 *
 *  Created: Tue Jun 07 14:06:56 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_BASEAPP_INIT_OPTIONS_H_
#define __LIBS_BASEAPP_INIT_OPTIONS_H_

#include <logging/logger.h>
#include <utils/system/dynamic_module/module.h>

namespace fawkes {
  namespace runtime {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class InitOptions
{
 public:
  InitOptions(const char *basename);
  InitOptions(int argc, char **argv);
  InitOptions(const InitOptions &options);
  ~InitOptions();

  InitOptions &  operator=(const InitOptions &options);

  InitOptions &  net_tcp_port(unsigned short int port);
  InitOptions &  net_service_name(const char *service_name);
  InitOptions &  daemonize(bool daemonize,
			   bool kill = false, bool status = false,
			   const char *pid_file = 0);
  InitOptions &  loggers(const char *loggers);
  InitOptions &  log_level(Logger::LogLevel log_level);
  InitOptions &  show_help(bool show_help);
  InitOptions &  user(const char *username);
  InitOptions &  group(const char *groupname);
  InitOptions &  config_file(const char *config_file);
  InitOptions &  bb_cleanup(bool bb_cleanup);
  InitOptions &  init_plugin_cache(bool init_plugin_cache);
  InitOptions &  load_plugins(const char *plugin_list);
  InitOptions &  default_plugin(const char *default_plugin);
  InitOptions &  plugin_module_flags(Module::ModuleFlags flags);
  InitOptions &  default_signal_handlers(bool enable);

  const char *basename() const;

  bool has_net_tcp_port() const;
  unsigned short int net_tcp_port() const;
  bool has_net_service_name() const;
  const char * net_service_name() const;

  bool has_load_plugin_list() const;
  const char * load_plugin_list() const;
  const char * default_plugin() const;
  

  bool has_loggers() const;
  const char *  loggers() const;
  Logger::LogLevel log_level() const;

  bool show_help() const;
  bool bb_cleanup() const;
  bool init_plugin_cache() const;

  bool daemonize() const;
  bool daemonize_kill() const;
  bool daemonize_status() const;
  const char *  daemon_pid_file() const;


  bool has_username() const;
  const char * username() const;
  bool has_groupname() const;
  const char * groupname() const;

  const char * config_file() const;

  Module::ModuleFlags plugin_module_flags() const;

  bool  default_signal_handlers() const;

 private:
  char               *__basename;

  bool                __has_net_tcp_port;
  unsigned short int  __net_tcp_port;

  bool                __has_load_plugin_list;
  char               *__load_plugin_list;
  char               *__default_plugin;

  bool                __has_loggers;
  char               *__loggers;
  Logger::LogLevel    __log_level;

  bool                __has_net_service_name;
  char               *__net_service_name;

  bool                __has_username;
  char               *__username;
  bool                __has_groupname;
  char               *__groupname;

  char               *__config_file;

  bool                __daemonize;
  char               *__daemon_pid_file;
  bool                __daemonize_kill;
  bool                __daemonize_status;

  bool                __show_help;
  bool                __bb_cleanup;

  bool                __init_plugin_cache;
  Module::ModuleFlags __plugin_module_flags;
  bool                __default_signal_handlers;
  
};


} // end namespace runtime
} // end namespace fawkes

#endif
