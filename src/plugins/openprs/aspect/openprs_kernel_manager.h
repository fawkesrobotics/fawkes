
/***************************************************************************
 *  openprs_kernel_manager.h - OpenPRS kernel manager
 *
 *  Created: Mon Aug 18 15:12:57 2014
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

#ifndef __PLUGINS_OPENPRS_ASPECT_OPENPRS_ENV_MANAGER_H_
#define __PLUGINS_OPENPRS_ASPECT_OPENPRS_ENV_MANAGER_H_

#include <core/utils/lockptr.h>
#include <string>
#include <map>
#include <list>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class Clock;
class Configuration;
class SubProcess;

class OpenPRSKernelManager
{
 public:
  OpenPRSKernelManager(const std::string &server_host, unsigned short server_tcp_port,
		       const std::string &mp_host, unsigned short mp_tcp_port,
		       Logger *logger, Clock *clock, Configuration *config);
  virtual ~OpenPRSKernelManager();

  void create_kernel(const std::string &kernel_name, bool use_xoprs,
		     std::list<std::string> &extra_data_path, bool utils_gdb_delay);
  void destroy_kernel(const std::string &kernel_name);

  std::list<std::string> kernels() const;

  /** Get oprs-server hostname.
   * @return hostname where oprs-server is running */
  const std::string &  server_host() const
  { return server_host_; }

  /** Get oprs-server TCP port.
   * @return TCP port where oprs-server is listening */
  unsigned short server_port() const
  { return server_port_; }

  /** Get mp-oprs hostname.
   * @return hostname where mp-oprs is running */
  const std::string &  mp_host() const
  { return mp_host_; }

  /** Get mp-oprs TCP port.
   * @return TCP port where mp-oprs is listening */
  unsigned short mp_port() const
  { return mp_port_; }

 private:
  const std::string    server_host_;
  const unsigned short server_port_;
  const std::string    mp_host_;
  const unsigned short mp_port_;

  Logger        *logger_;
  Clock         *clock_;
  Configuration *config_;

  std::map<std::string, fawkes::SubProcess *> kernels_;
};

} // end namespace fawkes

#endif
