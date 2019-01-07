
/***************************************************************************
 *  log.h - XML-RPC methods related to logging
 *
 *  Created: Mon Aug 31 20:35:32 2009
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

#ifndef _PLUGINS_XMLRPC_METHODS_LOG_H_
#define _PLUGINS_XMLRPC_METHODS_LOG_H_

#include <xmlrpc-c/registry.hpp>

#include <logging/logger.h>

namespace fawkes {
  class CacheLogger;
}

class XmlRpcLogMethods {
 public:
  XmlRpcLogMethods(std::shared_ptr<xmlrpc_c::registry> registry,
                   fawkes::CacheLogger *cache_logger,
                   fawkes::Logger *logger);
  ~XmlRpcLogMethods();

  class log_entries : public xmlrpc_c::method {
   public:
    log_entries(fawkes::CacheLogger *logger);
    virtual ~log_entries();
    virtual void execute(xmlrpc_c::paramList const& params,
			 xmlrpc_c::value *   const  result);
   private:
    fawkes::CacheLogger *cache_logger_;;
  };

  class log_get_size : public xmlrpc_c::method {
   public:
    log_get_size(fawkes::CacheLogger *logger);
    virtual ~log_get_size();
    virtual void execute(xmlrpc_c::paramList const& params,
			 xmlrpc_c::value *   const  result);
   private:
    fawkes::CacheLogger *cache_logger_;;
  };

  class log_set_size : public xmlrpc_c::method {
   public:
    log_set_size(fawkes::CacheLogger *cache_logger);
    virtual ~log_set_size();
    virtual void execute(xmlrpc_c::paramList const& params,
			 xmlrpc_c::value *   const  result);
   private:
    fawkes::CacheLogger *cache_logger_;;
  };

  class log_log : public xmlrpc_c::method {
   public:
    log_log(fawkes::Logger *logger, fawkes::Logger::LogLevel log_level);
    virtual ~log_log();
    virtual void execute(xmlrpc_c::paramList const& params,
			 xmlrpc_c::value *   const  result);
   private:
    fawkes::Logger           *logger_;
    fawkes::Logger::LogLevel  log_level_;
  };

 private:
  std::shared_ptr<xmlrpc_c::registry> xmlrpc_registry_;

  fawkes::Logger        *logger_;
  fawkes::CacheLogger   *cache_logger_;
	std::unique_ptr<log_entries>   log_entries_;
	std::unique_ptr<log_get_size>  log_get_size_;
	std::unique_ptr<log_set_size>  log_set_size_;
	std::unique_ptr<log_log>       log_log_debug_;
	std::unique_ptr<log_log>       log_log_info_;
	std::unique_ptr<log_log>       log_log_warn_;
	std::unique_ptr<log_log>       log_log_error_;
};

#endif
