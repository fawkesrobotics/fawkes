
/***************************************************************************
 *  clips_kernel_manager.cpp - OpenPRS kernel manager
 *
 *  Created: Mon Aug 18 15:20:20 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include <plugins/openprs/aspect/openprs_kernel_manager.h>
#include <utils/sub_process/proc.h>
#include <utils/misc/string_commands.h>
#include <logging/logger.h>
#include <config/config.h>
#include <utils/time/time.h>
#include <utils/misc/string_split.h>

#include <boost/format.hpp>

#include <cstring>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class OpenPRSKernelManager <plugins/openprs/aspect/openprs_kernel_manager.h>
 * OpenPRS kernel manager.
 * The OpenPRS kernel manager creates and maintains OpenPRS
 * kernels and provides them to the OpenPRS aspects.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param server_host OpenPRS server hostname
 * @param server_tcp_port TCP port where OpenPRS server listens on
 * @param mp_host OpenPRS message passer hostname
 * @param mp_tcp_port TCP port where OpenPRS message passer listens on
 * @param logger logger to log messages from created kernels
 * @param clock clock to get time from for (now)
 * @param config configuration
 */
OpenPRSKernelManager::OpenPRSKernelManager(const std::string &server_host, unsigned short server_tcp_port,
					   const std::string &mp_host, unsigned short mp_tcp_port,
					   Logger *logger, Clock *clock, Configuration *config)
  : server_host_(server_host), server_port_(server_tcp_port),
    mp_host_(mp_host), mp_port_(mp_tcp_port)
{
  logger_ = logger;
  clock_  = clock;
  config_ = config;
}

/** Destructor. */
OpenPRSKernelManager::~OpenPRSKernelManager()
{
}


/** Create a new kernel.
 * The kernel is registered internally under the specified name.
 * It must be destroyed when done with it. Only a single kernel
 * can be created for a particular kernel name.
 * @param kernel_name name by which to register kernel
 * @param use_xoprs run X-OPRS (with graphical user interface) instead
 * of oprs.
 * @param extra_data_path extra directories to add to the OPRS_DATA_PATH
 * environment variable which should be searched for files.
 * @param utils_gdb_delay if true, will set the FAWKES_OPRS_GDB_DELAY environment
 * variable to "true". If mod_utils is loaded it will wait for 10 seconds
 * and print a gdb command to start debugging the kernel process.
 */
void
OpenPRSKernelManager::create_kernel(const std::string &kernel_name, bool use_xoprs,
				    std::list<std::string> &extra_data_path, bool utils_gdb_delay)
{
  if (kernels_.find(kernel_name) != kernels_.end()) {
    throw Exception("OpenPRS kernel '%s' already exists", kernel_name.c_str());
  }

  std::string server_port = boost::str(boost::format("%u") % server_port_);
  std::string mp_port     = boost::str(boost::format("%u") % mp_port_);

  const char *argv[] = { use_xoprs ? "xoprs" : "oprs",
			 "-s", server_host_.c_str(), "-i", server_port.c_str(),
			 "-m", mp_host_.c_str(), "-j", mp_port.c_str(),
			 "-l", "lower",
			 "-n", kernel_name.c_str(),
			 NULL };

  std::list<std::string> data_path;
  try {
    if (config_->is_list("/openprs/kernels/data-path")) {
      std::vector<std::string> pl =
	config_->get_strings("/openprs/kernels/data-path");
      std::for_each(pl.begin(), pl.end(), [&data_path](std::string &p){ data_path.push_back(p); });
    } else {
      std::string cfg_data_path =
	config_->get_string("/openprs/kernels/data-path");
      data_path = str_split_list(cfg_data_path, ':');
    }
  } catch (Exception &e) {} // ignored
  std::list<std::string>::iterator ins_pos = data_path.begin();
  for (auto p : extra_data_path) {
    ins_pos = data_path.insert(ins_pos, p);
  }
  const std::string env_HOME = getenv("HOME");
  for (auto &p : data_path) {
    std::string::size_type pos = 0;
    while ((pos = p.find("$HOME", pos)) != std::string::npos) {
      p.replace(pos, 5, env_HOME);
      pos += env_HOME.length();
    }

    if ((pos = p.find("@BASEDIR@")) != std::string::npos) {
      p.replace(pos, 9, BASEDIR);
    }
    if ((pos = p.find("@FAWKES_BASEDIR@")) != std::string::npos) {
      p.replace(pos, 16, FAWKES_BASEDIR);
    }
    if ((pos = p.find("@RESDIR@")) != std::string::npos) {
      p.replace(pos, 8, RESDIR);
    }
    if ((pos = p.find("@CONFDIR@")) != std::string::npos) {
      p.replace(pos, 9, CONFDIR);
    }
  }
  std::string oprs_data_path = str_join(data_path, ':');

  const char *envp_path_ext[] = { "LD_LIBRARY_PATH", OPENPRS_MOD_DIR,
				  "OPRS_DATA_PATH", oprs_data_path.c_str(), NULL };
  std::vector<std::string> envp_v = envp_copy_expand(environ, envp_path_ext);

  envp_v.push_back(boost::str(boost::format("FAWKES_OPRS_GDB_DELAY=%s") %
			      (utils_gdb_delay ? "true" : "false")));

  const char *envp[envp_v.size() + 1];
  for (unsigned int i = 0; i < envp_v.size(); ++i) {
    envp[i] = envp_v[i].c_str();
    if (envp_v[i].find("OPRS_DATA_PATH=") == 0) {
      logger_->log_info("OpenPRSKernelMgr", "%s data path: %s", kernel_name.c_str(), envp[i]);
    }
  }
  envp[envp_v.size()] = NULL;

  std::string command = command_args_tostring(argv);
  logger_->log_info("OpenPRSKernelMgr", "Running:  %s", command.c_str());

  std::string progname = std::string(use_xoprs ? "XOPRS" : "OPRS") + "-" + kernel_name;

  SubProcess *oprs = NULL;
  std::string oprs_error;
  try {
    oprs = new SubProcess(progname.c_str(), argv[0], argv, (const char **)envp, logger_);
  } catch (Exception &e) {
    oprs = NULL;
    oprs_error = e.what_no_backtrace();
  }

  if (oprs) {
    // give some time for OpenPRS to come up
    usleep(500000);

    kernels_[kernel_name] = oprs;
  } else {
    throw Exception("Failed to initialize OpenPRS kernel '%s' (%s)",
		    kernel_name.c_str(), oprs_error.c_str());
  }
}

/** Destroy the named kernel.
 * Only ever destroy kernels which you have created yourself.
 * @param kernel_name name of the kernel to destroy
 */
void
OpenPRSKernelManager::destroy_kernel(const std::string &kernel_name)
{
  if (kernels_.find(kernel_name) != kernels_.end()) {
    delete kernels_[kernel_name];
    kernels_.erase(kernel_name);
  }
}


/** Get map of kernels.
 * @return map from kernel name to kernel lock ptr
 */
std::list<std::string>
OpenPRSKernelManager::kernels() const
{
  std::list<std::string> rv;
  for (auto k : kernels_) {
    rv.push_back(k.first);
  }
  return rv;
}



} // end namespace fawkes
