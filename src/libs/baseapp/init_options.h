
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

#ifndef _LIBS_BASEAPP_INIT_OPTIONS_H_
#define _LIBS_BASEAPP_INIT_OPTIONS_H_

#include <logging/logger.h>
#include <utils/system/dynamic_module/module.h>

namespace fawkes {
namespace runtime {

class InitOptions
{
public:
	InitOptions(const char *basename);
	InitOptions(int argc, char **argv);
	InitOptions(const InitOptions &options);
	~InitOptions();

	InitOptions &operator=(const InitOptions &options);

	InitOptions &net_tcp_port(unsigned short int port);
	InitOptions &net_service_name(const char *service_name);
	InitOptions &
	             daemonize(bool daemonize, bool kill = false, bool status = false, const char *pid_file = 0);
	InitOptions &loggers(const char *loggers);
	InitOptions &log_level(Logger::LogLevel log_level);
	InitOptions &show_help(bool show_help);
	InitOptions &user(const char *username);
	InitOptions &group(const char *groupname);
	InitOptions &config_file(const char *config_file);
	InitOptions &bb_cleanup(bool bb_cleanup);
	InitOptions &init_plugin_cache(bool init_plugin_cache);
	InitOptions &load_plugins(const char *plugin_list);
	InitOptions &default_plugin(const char *default_plugin);
	InitOptions &plugin_module_flags(Module::ModuleFlags flags);
	InitOptions &default_signal_handlers(bool enable);

	const char *basename() const;

	bool               has_net_tcp_port() const;
	unsigned short int net_tcp_port() const;
	bool               has_net_service_name() const;
	const char *       net_service_name() const;

	bool        has_load_plugin_list() const;
	const char *load_plugin_list() const;
	const char *default_plugin() const;

	bool             has_loggers() const;
	const char *     loggers() const;
	Logger::LogLevel log_level() const;

	bool show_help() const;
	bool bb_cleanup() const;
	bool init_plugin_cache() const;

	bool        daemonize() const;
	bool        daemonize_kill() const;
	bool        daemonize_status() const;
	const char *daemon_pid_file() const;

	bool        has_username() const;
	const char *username() const;
	bool        has_groupname() const;
	const char *groupname() const;

	const char *config_file() const;

	Module::ModuleFlags plugin_module_flags() const;

	bool default_signal_handlers() const;

private:
	char *basename_;

	bool               has_net_tcp_port_;
	unsigned short int net_tcp_port_;

	bool  has_load_plugin_list_;
	char *load_plugin_list_;
	char *default_plugin_;

	bool             has_loggers_;
	char *           loggers_;
	Logger::LogLevel log_level_;

	bool  has_net_service_name_;
	char *net_service_name_;

	bool  has_username_;
	char *username_;
	bool  has_groupname_;
	char *groupname_;

	char *config_file_;

	bool  daemonize_;
	char *daemon_pid_file_;
	bool  daemonize_kill_;
	bool  daemonize_status_;

	bool show_help_;
	bool bb_cleanup_;

	bool                init_plugin_cache_;
	Module::ModuleFlags plugin_module_flags_;
	bool                default_signal_handlers_;
};

} // end namespace runtime
} // end namespace fawkes

#endif
