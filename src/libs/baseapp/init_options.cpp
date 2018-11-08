
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
  basename_ = strdup(basename);
  default_plugin_ = strdup("default");
  has_net_tcp_port_ = false;
  net_tcp_port_ = 0;
  has_loggers_ = false;
  loggers_ = NULL;
  log_level_ = Logger::LL_DEBUG;
  has_net_service_name_ = false;
  net_service_name_ = NULL;
  has_username_ = false;
  username_ = NULL;
  has_groupname_ = false;
  groupname_ = NULL;
  config_file_ = NULL;
  daemonize_ = false;
  daemon_pid_file_ = NULL;
  daemonize_kill_ = false;
  daemonize_status_ = false;
  show_help_ = false;
  bb_cleanup_ = false;
  default_signal_handlers_ = true;
  init_plugin_cache_ = true;
  has_load_plugin_list_ = false;
  load_plugin_list_ = NULL;
  plugin_module_flags_ = Module::MODULE_FLAGS_DEFAULT;
}


/** Copy constructor.
 * @param options options object to copy
 */
InitOptions::InitOptions(const InitOptions &options)
{
  basename_ = strdup(options.basename_);
  default_plugin_ = strdup(options.default_plugin_);
  net_tcp_port_ = 0;
  has_net_tcp_port_ = options.has_net_tcp_port_;
  if (has_net_tcp_port_) {
    net_tcp_port_ = options.net_tcp_port_;
  }
  loggers_ = NULL;
  has_loggers_ = options.has_loggers_;
  if (has_loggers_) {
    loggers_ = strdup(options.loggers_);
  }

  log_level_ = options.log_level_;

  net_service_name_ = NULL;
  has_net_service_name_ = options.has_net_service_name_;
  if (has_net_service_name_) {
    net_service_name_ = strdup(options.net_service_name_);    
  }

  username_ = NULL;
  has_username_ = options.has_username_;
  if (has_username_) {
    username_ = strdup(options.username_);    
  }
  groupname_ = NULL;
  has_groupname_ = options.has_groupname_;
  if (has_groupname_) {
    groupname_ = strdup(options.groupname_);    
  }

  config_file_ = NULL;
  if (options.config_file_) {
    config_file_ = strdup(options.config_file_);
  }
  daemonize_ = options.daemonize_;
  daemon_pid_file_ = NULL;
  if (daemonize_ && options.daemon_pid_file_) {
    daemon_pid_file_ = strdup(options.daemon_pid_file_);
  }
  daemonize_kill_ = options.daemonize_kill_;
  daemonize_status_ = options.daemonize_status_;
  show_help_ = options.show_help_;
  bb_cleanup_ = options.bb_cleanup_;
  default_signal_handlers_ = options.default_signal_handlers_;
  init_plugin_cache_ = options.init_plugin_cache_;
  load_plugin_list_ = NULL;
  has_load_plugin_list_ = options.has_load_plugin_list_;
  if (has_load_plugin_list_) {
    load_plugin_list_ = strdup(options.load_plugin_list_);
  }

  plugin_module_flags_ = options.plugin_module_flags_;
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

  basename_ = strdup(argp->program_name());
  default_plugin_ = strdup("default");

  has_net_tcp_port_ = argp->has_arg("P");
  if (has_net_tcp_port_) {
    net_tcp_port_ = argp->parse_int("P");
  }
  has_loggers_ = argp->has_arg("L");
  if (has_loggers_) {
    loggers_ = strdup(argp->arg("L"));
  }

  const char *tmp;
  log_level_ = Logger::LL_INFO;
  if ( argp->has_arg("d") ) {
    log_level_ = Logger::LL_DEBUG;
  } else if ( argp->has_arg("q") ) {
    log_level_ = Logger::LL_WARN;
    if ( (tmp = argp->arg("q")) != NULL ) {
      for (unsigned int i = 0; i < strlen(tmp); ++i) {
	if ( tmp[i] == 'q' ) {
	  switch (log_level_) {
	  case Logger::LL_INFO:  log_level_ = Logger::LL_WARN; break;
	  case Logger::LL_WARN:  log_level_ = Logger::LL_ERROR; break;
	  case Logger::LL_ERROR: log_level_ = Logger::LL_NONE; break;
	  default: break;
	  }
	}
      }
    }
  } else if ( (tmp = argp->arg("l")) != NULL ) {
    if ( strcmp(tmp, "debug") == 0 ) {
      log_level_ = Logger::LL_DEBUG;
    } else if ( strcmp(tmp, "info") == 0 ) {
      log_level_ = Logger::LL_INFO;
    } else if ( strcmp(tmp, "warn") == 0 ) {
      log_level_ = Logger::LL_WARN;
    } else if ( strcmp(tmp, "error") == 0 ) {
      log_level_ = Logger::LL_ERROR;
    } else if ( strcmp(tmp, "none") == 0 ) {
      log_level_ = Logger::LL_NONE;
    }
  }

  has_net_service_name_ = argp->has_arg("net-service-name");
  if (has_net_service_name_) {
    net_service_name_ = strdup(argp->arg("net-service-name"));
  } else {
    net_service_name_ = NULL;
  }

  has_username_ = argp->has_arg("u");
  if (has_username_) {
    username_ = strdup(argp->arg("u"));
  } else {
    username_ = NULL;
  }

  has_groupname_ = argp->has_arg("u");
  if (has_groupname_) {
    groupname_ = strdup(argp->arg("u"));
  } else {
    groupname_ = NULL;
  }


  config_file_ = NULL;
  if (argp->arg("c")) {
    config_file_ = strdup(argp->arg("c"));
  }

  daemonize_ = argp->has_arg("D");
  daemonize_kill_ = daemonize_ && argp->has_arg("k");
  daemonize_status_ = daemonize_ && argp->has_arg("s");
  daemon_pid_file_ = NULL;
  if (daemonize_ && argp->arg("D")) {
    daemon_pid_file_ = strdup(argp->arg("D"));
  } else {
    daemon_pid_file_ = NULL;
  }
  show_help_ = argp->has_arg("h");
  bb_cleanup_ = argp->has_arg("C");

  has_load_plugin_list_ = argp->has_arg("p") || argp->num_items() > 0;
  if (has_load_plugin_list_) {
    uint len = 0;
    for (uint i = 0; i < argp->items().size(); i++) {
      len += strlen(argp->items()[i]);
    }
    if (argp->has_arg("p")) {
      len += strlen(argp->arg("p") + 1);
    }
    char res[len + argp->items().size()];
    if (argp->has_arg("p") && argp->num_items() > 0) {
      sprintf(res, "%s,%s,", argp->arg("p"), argp->items()[0]);
    } else if (argp->has_arg("p")) {
      sprintf(res, "%s", argp->arg("p"));
    } else {
      sprintf(res, "%s", argp->items()[0]);
    }
    for (uint i = 1; i < argp->items().size(); i++) {
      char *tmp = strdup(res);
      sprintf(res, "%s,%s", tmp, argp->items()[i]);
      free(tmp);
    }
    load_plugin_list_ = strdup(res);
  } else {
    load_plugin_list_ = NULL;
  }

  init_plugin_cache_ = true;
  plugin_module_flags_ = Module::MODULE_FLAGS_DEFAULT;
  default_signal_handlers_ = true;
}


/** Destructor. */
InitOptions::~InitOptions()
{
  free(basename_);
  free(default_plugin_);
  if (has_loggers_)           free(loggers_);
  if (has_net_service_name_)  free(net_service_name_);
  if (has_username_)          free(username_);
  if (has_groupname_)         free(groupname_);
  if (has_load_plugin_list_)  free(load_plugin_list_);
  if (config_file_)           free(config_file_);
  if (daemon_pid_file_)       free(daemon_pid_file_);
}


/** Assignment operator.
 * @param options options object to copy
 * @return reference to this instance
 */
InitOptions &
InitOptions::operator=(const InitOptions &options)
{
  free(basename_);
  basename_ = strdup(options.basename_);
  free(default_plugin_);
  default_plugin_ = strdup(options.default_plugin_);
  net_tcp_port_ = 0;
  has_net_tcp_port_ = options.has_net_tcp_port_;
  if (has_net_tcp_port_) {
    net_tcp_port_ = options.net_tcp_port_;
  }
  if (has_loggers_) {
    has_loggers_ = false;
    free(loggers_);
    loggers_ = NULL;
  }
  has_loggers_ = options.has_loggers_;
  if (has_loggers_) {
    loggers_ = strdup(options.loggers_);
  }

  log_level_ = options.log_level_;

  if (has_net_service_name_) {
    has_net_service_name_ = false;
    free(net_service_name_);
    net_service_name_ = NULL;
  }
  has_net_service_name_ = options.has_net_service_name_;
  if (has_net_service_name_) {
    net_service_name_ = strdup(options.net_service_name_);    
  }

  if (has_username_) {
    has_username_ = false;
    free(username_);
    username_ = NULL;
  }
  has_username_ = options.has_username_;
  if (has_username_) {
    username_ = strdup(options.username_);    
  }

  if (has_groupname_) {
    has_groupname_ = false;
    free(groupname_);
    groupname_ = NULL;
  }
  groupname_ = NULL;
  has_groupname_ = options.has_groupname_;
  if (has_groupname_) {
    groupname_ = strdup(options.groupname_);    
  }

  if (config_file_) {
    free(config_file_);
    config_file_ = NULL;
  }
  if (options.config_file_) {
    config_file_ = strdup(options.config_file_);
  }

  daemonize_ = options.daemonize_;
  if (daemon_pid_file_) {
    free(daemon_pid_file_);
    daemon_pid_file_ = NULL;
  }
  if (daemonize_ && options.daemon_pid_file_) {
    daemon_pid_file_ = strdup(options.daemon_pid_file_);
  }
  daemonize_kill_ = options.daemonize_kill_;
  daemonize_status_ = options.daemonize_status_;
  show_help_ = options.show_help_;
  bb_cleanup_ = options.bb_cleanup_;

  if (load_plugin_list_) {
    free(load_plugin_list_);
    load_plugin_list_ = NULL;
  }
  has_load_plugin_list_ = options.has_load_plugin_list_;
  if (has_load_plugin_list_) {
    load_plugin_list_ = strdup(options.load_plugin_list_);
  }

  init_plugin_cache_ = options.init_plugin_cache_;
  plugin_module_flags_ = options.plugin_module_flags_;
  default_signal_handlers_ = options.default_signal_handlers_;

  return *this;
}


/** Set additional default plugin name.
 * @param default_plugin_ additional default plugin name
 * @return reference to this instance
 */
InitOptions &
InitOptions::default_plugin(const char *default_plugin_)
{
  free(default_plugin_);
  default_plugin_ = strdup(default_plugin_);
  return *this;
}


/** Set Fawkes network TCP port.
 * @param port TCP port
 * @return reference to this instance
 */
InitOptions &
InitOptions::net_tcp_port(unsigned short int port)
{
  has_net_tcp_port_ = true;
  net_tcp_port_ = port;
  return *this;
}

/** Set Fawkes network service name.
 * @param service_name service name
 * @return reference to this instance
 */
InitOptions &
InitOptions::net_service_name(const char *service_name)
{
  if (has_net_service_name_) {
    has_net_service_name_ = false;
    free(net_service_name_);
  }
  if (service_name) {
    has_net_service_name_ = true;
    net_service_name_ = strdup(service_name);
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
  daemonize_ = daemonize;
  daemonize_kill_ = daemonize && kill;
  daemonize_status_ = daemonize && status;
  if (daemonize && pid_file) {
    daemon_pid_file_ = strdup(pid_file);
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
  if (has_loggers_) {
    has_loggers_ = false;
    free(loggers_);
  }
  if (loggers) {
    has_loggers_ = true;
    loggers_ = strdup(loggers);
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
  log_level_ = log_level;
  return *this;
}

/** Set to show help.
 * @param show_help true to request showing help information, false otherwise
 * @return reference to this instance
 */
InitOptions &
InitOptions::show_help(bool show_help)
{
  show_help_ = show_help;
  return *this;
}


/** Enable or disable plugin cache initialization.
 * @param init_cache true to trigger plugin cache initialization, false to disable
 * @return reference to this instance
 */
InitOptions &
InitOptions::init_plugin_cache(bool init_cache)
{
  init_plugin_cache_ = init_cache;
  return *this;
}

/** Set user name to run as.
 * @param username user name to run as
 * @return reference to this instance
 */
InitOptions &
InitOptions::user(const char *username)
{
  if (has_username_) {
    has_username_ = false;
    free(username_);
  }
  if (username) {
    has_username_ = true;
    username_ = strdup(username);
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
  if (has_load_plugin_list_) {
    has_load_plugin_list_ = false;
    free(load_plugin_list_);
    load_plugin_list_ = NULL;
  }
  if (plugin_list) {
    has_load_plugin_list_ = true;
    load_plugin_list_ = strdup(plugin_list);
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
  if (has_groupname_) {
    has_groupname_ = false;
    free(groupname_);
  }
  if (groupname) {
    has_groupname_ = true;
    groupname_ = strdup(groupname);
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
  if (config_file_) {
    free(config_file_);
    config_file_ = NULL;
  }
  if (config_file) {
    config_file_ = strdup(config_file);
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
  bb_cleanup_ = bb_cleanup;
  return *this;
}


/** Set module flags.
 * @param flags flags to open plugin modules with
 * @return reference to this instance
 */
InitOptions &
InitOptions::plugin_module_flags(Module::ModuleFlags flags)
{
  plugin_module_flags_ = flags;
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
  default_signal_handlers_ = enable;
  return *this;
}


/** Get program basename.
 * @return program base name
 */
const char *
InitOptions::basename() const
{
  return basename_;
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
  return default_plugin_;
}


/** Check if TCP port has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_net_tcp_port() const
{
  return has_net_tcp_port_;
}

/** Get Fawkes network TCP port.
 * @return Fawkes network TCP port
 */
unsigned short int
InitOptions::net_tcp_port() const
{
  return net_tcp_port_;
}

/** Check if network service name has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_net_service_name() const
{
  return has_net_service_name_;
}

/** Get network service name.
 * @return network service name
 */
const char *
InitOptions::net_service_name() const
{
  return net_service_name_;
}

/** Check if plugin load list has been set.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_load_plugin_list() const
{
  return has_load_plugin_list_;
}

/** Get plugin load list.
 * @return plugin load list
 */
const char *
InitOptions::load_plugin_list() const
{
  return load_plugin_list_;
}

/** Check if logger string has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_loggers() const
{
  return has_loggers_;
}

/** Get logger string.
 * @return logger stirng
 */
const char *
InitOptions::loggers() const
{
  return loggers_;
}

/** Get log level.
 * @return log level
 */
Logger::LogLevel
InitOptions::log_level() const
{
  return log_level_;
}

/** Check if help has been requested.
 * @return true if help has been requested, false otherwise
 */
bool
InitOptions::show_help() const
{
  return show_help_;
}

/** Check if blackboard cleanup has been requested.
 * @return true if blackboard cleanup has been requested, false otherwise
 */
bool
InitOptions::bb_cleanup() const
{
  return bb_cleanup_;
}


/** Check if plugin cache initialization has been requested.
 * @return true if plugin cache initialization has been requested, false otherwise
 */
bool
InitOptions::init_plugin_cache() const
{
  return init_plugin_cache_;
}


/** Check if default signal handlers should be enabled.
 * @return true if default signal handlers have been requested, false otherwise
 */
bool
InitOptions::default_signal_handlers() const
{
  return default_signal_handlers_;
}

/** Check if daemonization has been requested.
 * @return true if daemonization has been requested, false otherwise
 */
bool
InitOptions::daemonize() const
{
  return daemonize_;
}

/** Check if killing of daemon has been requested.
 * @return true if killing of daemon has been requested, false otherwise
 */
bool
InitOptions::daemonize_kill() const
{
  return daemonize_kill_;
}

/** Check if status of daemon has been requested.
 * @return true if status of daemon has been requested, false otherwise
 */
bool
InitOptions::daemonize_status() const
{
  return daemonize_status_;
}


/** Get daemon PID file.
 * @return daemon PID file path
 */
const char *
InitOptions::daemon_pid_file() const
{
  return daemon_pid_file_;
}


/** Check if user name has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_username() const
{
  return has_username_;
}


/** Get user name to run as.
 * @return user name to run as
 */
const char *
InitOptions::username() const
{
  return username_;
}

/** Check if group name has been passed.
 * @return true if the parameter has been set, false otherwise
 */
bool
InitOptions::has_groupname() const
{
  return has_groupname_;
}

/** Get group name to run as.
 * @return group name to run as
 */
const char *
InitOptions::groupname() const
{
  return groupname_;
}


/** Get config file path.
 * @return config file path
 */
const char *
InitOptions::config_file() const
{
  return config_file_;
}


/** Get plugin module flags.
 * @return plugin module flags
 */
Module::ModuleFlags
InitOptions::plugin_module_flags() const
{
  return plugin_module_flags_;
}


} // end namespace runtime
} // end namespace fawkes
