
/***************************************************************************
 *  init_options.cpp - Fawkes run-time initialization options
 *
 *  Created: Tue Jun 07 14:19:56 2011
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

#include <baseapp/init_options.h>
#include <baseapp/run.h>
#include <utils/system/argparser.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {
  namespace runtime {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif


/** @class InitOptions <baseapp/init_options.h>
 * Initialization options class.
 * This class provides a container for initialization options that can be
 * passed to the Fawkes runtime. It uses the named parameter idiom which
 * allows to set only the parameters which divert from the default value.
 * @author Tim Niemueller
 */


/** Constructor.
 * Initializes the default options.
 * @param basename program base name
 */
InitOptions::InitOptions(const char *basename)
{
  __basename = strdup(basename);
  __default_plugin = strdup("default");
  __has_net_tcp_port = false;
  __net_tcp_port = 0;
  __has_loggers = false;
  __loggers = NULL;
  __log_level = Logger::LL_DEBUG;
  __has_net_service_name = false;
  __net_service_name = NULL;
  __has_username = false;
  __username = NULL;
  __has_groupname = false;
  __groupname = NULL;
  __config_file = NULL;
  __daemonize = false;
  __daemon_pid_file = NULL;
  __daemonize_kill = false;
  __daemonize_status = false;
  __show_help = false;
  __bb_cleanup = false;
  __default_signal_handlers = true;
  __init_plugin_cache = true;
  __has_load_plugin_list = false;
  __load_plugin_list = NULL;
  __plugin_module_flags = Module::MODULE_FLAGS_DEFAULT;
}


/** Copy constructor.
 * @param options options object to copy
 */
InitOptions::InitOptions(const InitOptions &options)
{
  __basename = strdup(options.__basename);
  __default_plugin = strdup(options.__default_plugin);
  __net_tcp_port = 0;
  __has_net_tcp_port = options.__has_net_tcp_port;
  if (__has_net_tcp_port) {
    __net_tcp_port = options.__net_tcp_port;
  }
  __loggers = NULL;
  __has_loggers = options.__has_loggers;
  if (__has_loggers) {
    __loggers = strdup(options.__loggers);
  }

  __log_level = options.__log_level;

  __net_service_name = NULL;
  __has_net_service_name = options.__has_net_service_name;
  if (__has_net_service_name) {
    __net_service_name = strdup(options.__net_service_name);    
  }

  __username = NULL;
  __has_username = options.__has_username;
  if (__has_username) {
    __username = strdup(options.__username);    
  }
  __groupname = NULL;
  __has_groupname = options.__has_groupname;
  if (__has_groupname) {
    __groupname = strdup(options.__groupname);    
  }

  __config_file = NULL;
  if (options.__config_file) {
    __config_file = strdup(options.__config_file);
  }
  __daemonize = options.__daemonize;
  __daemon_pid_file = NULL;
  if (__daemonize && options.__daemon_pid_file) {
    __daemon_pid_file = strdup(options.__daemon_pid_file);
  }
  __daemonize_kill = options.__daemonize_kill;
  __daemonize_status = options.__daemonize_status;
  __show_help = options.__show_help;
  __bb_cleanup = options.__bb_cleanup;
  __default_signal_handlers = options.__default_signal_handlers;
  __init_plugin_cache = options.__init_plugin_cache;
  __load_plugin_list = NULL;
  __has_load_plugin_list = options.__has_load_plugin_list;
  if (__has_load_plugin_list) {
    __load_plugin_list = strdup(options.__load_plugin_list);
  }

  __plugin_module_flags = options.__plugin_module_flags;
}


/** Constructor from arguments.
 * Initializes the options from arguments passed from the command line.
 * @param argc number of elements in @p argv
 * @param argv argument array
 */
InitOptions::InitOptions(int argc, char **argv)
{

  option long_options[] = {
    {"net-service-name", 1, 0, 0},
      {0, 0, 0, 0}
  };

  fawkes::runtime::argument_parser =
    new ArgumentParser(argc, argv,
		       "hCc:dq::l:L:p:P:u:g:D::ks",
		       long_options);

  ArgumentParser *argp = fawkes::runtime::argument_parser;

  __basename = strdup(argp->program_name());
  __default_plugin = strdup("default");

  __has_net_tcp_port = argp->has_arg("P");
  if (__has_net_tcp_port) {
    __net_tcp_port = argp->parse_int("P");
  }
  __has_loggers = argp->has_arg("L");
  if (__has_loggers) {
    __loggers = strdup(argp->arg("L"));
  }

  const char *tmp;
  __log_level = Logger::LL_INFO;
  if ( argp->has_arg("d") ) {
    __log_level = Logger::LL_DEBUG;
  } else if ( argp->has_arg("q") ) {
    __log_level = Logger::LL_WARN;
    if ( (tmp = argp->arg("q")) != NULL ) {
      for (unsigned int i = 0; i < strlen(tmp); ++i) {
	if ( tmp[i] == 'q' ) {
	  switch (__log_level) {
	  case Logger::LL_INFO:  __log_level = Logger::LL_WARN; break;
	  case Logger::LL_WARN:  __log_level = Logger::LL_ERROR; break;
	  case Logger::LL_ERROR: __log_level = Logger::LL_NONE; break;
	  default: break;
	  }
	}
      }
    }
  } else if ( (tmp = argp->arg("l")) != NULL ) {
    if ( strcmp(tmp, "debug") == 0 ) {
      __log_level = Logger::LL_DEBUG;
    } else if ( strcmp(tmp, "info") == 0 ) {
      __log_level = Logger::LL_INFO;
    } else if ( strcmp(tmp, "warn") == 0 ) {
      __log_level = Logger::LL_WARN;
    } else if ( strcmp(tmp, "error") == 0 ) {
      __log_level = Logger::LL_ERROR;
    } else if ( strcmp(tmp, "none") == 0 ) {
      __log_level = Logger::LL_NONE;
    }
  }

  __has_net_service_name = argp->has_arg("net-service-name");
  if (__has_net_service_name) {
    __net_service_name = strdup(argp->arg("net-service-name"));
  } else {
    __net_service_name = NULL;
  }

  __has_username = argp->has_arg("u");
  if (__has_username) {
    __username = strdup(argp->arg("u"));
  } else {
    __username = NULL;
  }

  __has_groupname = argp->has_arg("u");
  if (__has_groupname) {
    __groupname = strdup(argp->arg("u"));
  } else {
    __groupname = NULL;
  }


  __config_file = NULL;
  if (argp->arg("c")) {
    __config_file = strdup(argp->arg("c"));
  }

  __daemonize = argp->has_arg("D");
  __daemonize_kill = __daemonize && argp->has_arg("k");
  __daemonize_status = __daemonize && argp->has_arg("s");
  __daemon_pid_file = NULL;
  if (__daemonize && argp->arg("D")) {
    __daemon_pid_file = strdup(argp->arg("D"));
  } else {
    __daemon_pid_file = NULL;
  }
  __show_help = argp->has_arg("h");
  __bb_cleanup = argp->has_arg("C");

  __has_load_plugin_list = argp->has_arg("p");
  if (__has_load_plugin_list) {
    __load_plugin_list = strdup(argp->arg("p"));
  } else {
    __load_plugin_list = NULL;
  }

  __init_plugin_cache = true;
  __plugin_module_flags = Module::MODULE_FLAGS_DEFAULT;
  __default_signal_handlers = true;
}


/** Destructor. */
InitOptions::~InitOptions()
{
  free(__basename);
  free(__default_plugin);
  if (__has_loggers)           free(__loggers);
  if (__has_net_service_name)  free(__net_service_name);
  if (__has_username)          free(__username);
  if (__has_groupname)         free(__groupname);
  if (__has_load_plugin_list)  free(__load_plugin_list);
  if (__config_file)           free(__config_file);
  if (__daemon_pid_file)       free(__daemon_pid_file);
}


/** Assignment operator.
 * @param options options object to copy
 * @return reference to this instance
 */
InitOptions &
InitOptions::operator=(const InitOptions &options)
{
  free(__basename);
  __basename = strdup(options.__basename);
  free(__default_plugin);
  __default_plugin = strdup(options.__default_plugin);
  __net_tcp_port = 0;
  __has_net_tcp_port = options.__has_net_tcp_port;
  if (__has_net_tcp_port) {
    __net_tcp_port = options.__net_tcp_port;
  }
  if (__has_loggers) {
    __has_loggers = false;
    free(__loggers);
    __loggers = NULL;
  }
  __has_loggers = options.__has_loggers;
  if (__has_loggers) {
    __loggers = strdup(options.__loggers);
  }

  __log_level = options.__log_level;

  if (__has_net_service_name) {
    __has_net_service_name = false;
    free(__net_service_name);
    __net_service_name = NULL;
  }
  __has_net_service_name = options.__has_net_service_name;
  if (__has_net_service_name) {
    __net_service_name = strdup(options.__net_service_name);    
  }

  if (__has_username) {
    __has_username = false;
    free(__username);
    __username = NULL;
  }
  __has_username = options.__has_username;
  if (__has_username) {
    __username = strdup(options.__username);    
  }

  if (__has_groupname) {
    __has_groupname = false;
    free(__groupname);
    __groupname = NULL;
  }
  __groupname = NULL;
  __has_groupname = options.__has_groupname;
  if (__has_groupname) {
    __groupname = strdup(options.__groupname);    
  }

  if (__config_file) {
    free(__config_file);
    __config_file = NULL;
  }
  if (options.__config_file) {
    __config_file = strdup(options.__config_file);
  }

  __daemonize = options.__daemonize;
  if (__daemon_pid_file) {
    free(__daemon_pid_file);
    __daemon_pid_file = NULL;
  }
  if (__daemonize && options.__daemon_pid_file) {
    __daemon_pid_file = strdup(options.__daemon_pid_file);
  }
  __daemonize_kill = options.__daemonize_kill;
  __daemonize_status = options.__daemonize_status;
  __show_help = options.__show_help;
  __bb_cleanup = options.__bb_cleanup;

  if (__load_plugin_list) {
    free(__load_plugin_list);
    __load_plugin_list = NULL;
  }
  __has_load_plugin_list = options.__has_load_plugin_list;
  if (__has_load_plugin_list) {
    __load_plugin_list = strdup(options.__load_plugin_list);
  }

  __init_plugin_cache = options.__init_plugin_cache;
  __plugin_module_flags = options.__plugin_module_flags;
  __default_signal_handlers = options.__default_signal_handlers;

  return *this;
}


/** Set additional default plugin name.
 * @param default_plugin_ additional default plugin name
 * @return reference to this instance
 */
InitOptions &
InitOptions::default_plugin(const char *default_plugin_)
{
  free(__default_plugin);
  __default_plugin = strdup(default_plugin_);
  return *this;
}


/** Set Fawkes network TCP port.
 * @param port TCP port
 * @return reference to this instance
 */
InitOptions &
InitOptions::net_tcp_port(unsigned short int port)
{
  __has_net_tcp_port = true;
  __net_tcp_port = port;
  return *this;
}

/** Set Fawkes network service name.
 * @param service_name service name
 * @return reference to this instance
 */
InitOptions &
InitOptions::net_service_name(const char *service_name)
{
  if (__has_net_service_name) {
    __has_net_service_name = false;
    free(__net_service_name);
  }
  if (service_name) {
    __has_net_service_name = true;
    __net_service_name = strdup(service_name);
  }
  return *this;
}

/** Set daemonization options.
 * @param daemonize daemonization requested
 * @param kill kill a running daemon
 * @param status print status about running daemon
 * @param pid_file path to file to write PID to
 * @return reference to this instance
 */
InitOptions &
InitOptions::daemonize(bool daemonize, bool kill, bool status,
		       const char *pid_file)
{
  __daemonize = daemonize;
  __daemonize_kill = daemonize && kill;
  __daemonize_status = daemonize && status;
  if (daemonize && pid_file) {
    __daemon_pid_file = strdup(pid_file);
  }
  return *this;
}

/** Set loggers.
 * @param loggers string of loggers
 * @return reference to this instance
 */
InitOptions &
InitOptions::loggers(const char *loggers)
{
  if (__has_loggers) {
    __has_loggers = false;
    free(__loggers);
  }
  if (loggers) {
    __has_loggers = true;
    __loggers = strdup(loggers);
  }
  return *this;
}

/** Set log level.
 * @param log_level desired log level
 * @return reference to this instance
 */
InitOptions &
InitOptions::log_level(Logger::LogLevel log_level)
{
  __log_level = log_level;
  return *this;
}

/** Set to show help.
 * @param show_help true to request showing help information, false otherwise
 * @return reference to this instance
 */
InitOptions &
InitOptions::show_help(bool show_help)
{
  __show_help = show_help;
  return *this;
}


/** Enable or disable plugin cache initialization.
 * @param init_cache true to trigger plugin cache initialization, false to disable
 * @return reference to this instance
 */
InitOptions &
InitOptions::init_plugin_cache(bool init_cache)
{
  __init_plugin_cache = init_cache;
  return *this;
}

/** Set user name to run as.
 * @param username user name to run as
 * @return reference to this instance
 */
InitOptions &
InitOptions::user(const char *username)
{
  if (__has_username) {
    __has_username = false;
    free(__username);
  }
  if (username) {
    __has_username = true;
    __username = strdup(username);
  }
  return *this;
}


/** Set list of plugins to load during startup.
 * @param plugin_list comma-separated list of names of plugins to load
 * @return reference to this instance
 */
InitOptions &
InitOptions::load_plugins(const char *plugin_list)
{
  if (__has_load_plugin_list) {
    __has_load_plugin_list = false;
    free(__load_plugin_list);
    __load_plugin_list = NULL;
  }
  if (plugin_list) {
    __has_load_plugin_list = true;
    __load_plugin_list = strdup(plugin_list);
  }
  return *this;
}


/** Set group name to run as.
 * @param groupname user name to run as
 * @return reference to this instance
 */
InitOptions &
InitOptions::group(const char *groupname)
{
  if (__has_groupname) {
    __has_groupname = false;
    free(__groupname);
  }
  if (groupname) {
    __has_groupname = true;
    __groupname = strdup(groupname);
  }
  return *this;
}

/** Set config file path.
 * @param config_file config file path
 * @return reference to this instance
 */
InitOptions &
InitOptions::config_file(const char *config_file)
{
  if (__config_file) {
    free(__config_file);
    __config_file = NULL;
  }
  if (config_file) {
    __config_file = strdup(config_file);
  }
  return *this;
}

/** Set blackboard cleanup.
 * @param bb_cleanup true to run blackboard cleanup, false otherwise
 * @return reference to this instance
 */
InitOptions &
InitOptions::bb_cleanup(bool bb_cleanup)
{
  __bb_cleanup = bb_cleanup;
  return *this;
}


/** Set module flags.
 * @param flags flags to open plugin modules with
 * @return reference to this instance
 */
InitOptions &
InitOptions::plugin_module_flags(Module::ModuleFlags flags)
{
  __plugin_module_flags = flags;
  return *this;
}


/** Set default signal handlers.
 * @param enable true to enable default signal handlers, false to disable. Note
 * that if you disable the signal handlers you must stop the Fawkes main thread
 * execution by yourself by some other means.
 * @return reference to this instance
 */
InitOptions &
InitOptions::default_signal_handlers(bool enable)
{
  __default_signal_handlers = enable;
  return *this;
}


/** Get program basename.
 * @return program base name
 */
const char *
InitOptions::basename() const
{
  return __basename;
}


/** Get name of default plugin.
 * This is usually the name of a meta plugin to load the appropriate
 * plugins.  It may have a specialized name on a specific robot
 * platform. It defaults to "default". Note that "default" is always
 * loaded to avoid confusion.
 * @return default plugin name
 */
const char *
InitOptions::default_plugin() const
{
  return __default_plugin;
}


/** Check if TCP port has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_net_tcp_port() const
{
  return __has_net_tcp_port;
}

/** Get Fawkes network TCP port.
 * @return Fawkes network TCP port
 */
unsigned short int
InitOptions::net_tcp_port() const
{
  return __net_tcp_port;
}

/** Check if network service name has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_net_service_name() const
{
  return __has_net_service_name;
}

/** Get network service name.
 * @return network service name
 */
const char *
InitOptions::net_service_name() const
{
  return __net_service_name;
}

/** Check if plugin load list has been set.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_load_plugin_list() const
{
  return __has_load_plugin_list;
}

/** Get plugin load list.
 * @return plugin load list
 */
const char *
InitOptions::load_plugin_list() const
{
  return __load_plugin_list;
}

/** Check if logger string has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_loggers() const
{
  return __has_loggers;
}

/** Get logger string.
 * @return logger stirng
 */
const char *
InitOptions::loggers() const
{
  return __loggers;
}

/** Get log level.
 * @return log level
 */
Logger::LogLevel
InitOptions::log_level() const
{
  return __log_level;
}

/** Check if help has been requested.
 * @return true if help has been requested, false otherwise
 */
bool
InitOptions::show_help() const
{
  return __show_help;
}

/** Check if blackboard cleanup has been requested.
 * @return true if blackboard cleanup has been requested, false otherwise
 */
bool
InitOptions::bb_cleanup() const
{
  return __bb_cleanup;
}


/** Check if plugin cache initialization has been requested.
 * @return true if plugin cache initialization has been requested, false otherwise
 */
bool
InitOptions::init_plugin_cache() const
{
  return __init_plugin_cache;
}


/** Check if default signal handlers should be enabled.
 * @return true if default signal handlers have been requested, false otherwise
 */
bool
InitOptions::default_signal_handlers() const
{
  return __default_signal_handlers;
}

/** Check if daemonization has been requested.
 * @return true if daemonization has been requested, false otherwise
 */
bool
InitOptions::daemonize() const
{
  return __daemonize;
}

/** Check if killing of daemon has been requested.
 * @return true if killing of daemon has been requested, false otherwise
 */
bool
InitOptions::daemonize_kill() const
{
  return __daemonize_kill;
}

/** Check if status of daemon has been requested.
 * @return true if status of daemon has been requested, false otherwise
 */
bool
InitOptions::daemonize_status() const
{
  return __daemonize_status;
}


/** Get daemon PID file.
 * @return daemon PID file path
 */
const char *
InitOptions::daemon_pid_file() const
{
  return __daemon_pid_file;
}


/** Check if user name has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_username() const
{
  return __has_username;
}


/** Get user name to run as.
 * @return user name to run as
 */
const char *
InitOptions::username() const
{
  return __username;
}

/** Check if group name has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_groupname() const
{
  return __has_groupname;
}

/** Get group name to run as.
 * @return group name to run as
 */
const char *
InitOptions::groupname() const
{
  return __groupname;
}


/** Get config file path.
 * @return config file path
 */
const char *
InitOptions::config_file() const
{
  return __config_file;
}


/** Get plugin module flags.
 * @return plugin module flags
 */
Module::ModuleFlags
InitOptions::plugin_module_flags() const
{
  return __plugin_module_flags;
}


} // end namespace runtime
} // end namespace fawkes
