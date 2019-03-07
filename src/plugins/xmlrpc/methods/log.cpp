
/***************************************************************************
 *  log.cpp - XML-RPC methods related to logging
 *
 *  Created: Mon Aug 31 20:50:37 2009
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

#include "log.h"

#include <logging/cache.h>

#include <xmlrpc-c/girerr.hpp>

using namespace fawkes;

/** @class XmlRpcLogMethods "log.h"
 * Wrapper class for logging related XML-RPC methods.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param registry XML registry, methods will be automatically registered
 * @param cache_logger cache logger to access recent log messages
 * @param logger logger to output messages
 */
XmlRpcLogMethods::XmlRpcLogMethods(std::shared_ptr<xmlrpc_c::registry> registry,
                                   fawkes::CacheLogger *               cache_logger,
                                   fawkes::Logger *                    logger)
{
	xmlrpc_registry_ = registry;
	cache_logger_    = cache_logger;
	logger_          = logger;
	log_entries_.reset(new log_entries(cache_logger));
	log_get_size_.reset(new log_get_size(cache_logger));
	log_set_size_.reset(new log_set_size(cache_logger));
	log_log_debug_.reset(new log_log(logger, fawkes::Logger::LL_DEBUG));
	log_log_info_.reset(new log_log(logger, fawkes::Logger::LL_INFO));
	log_log_warn_.reset(new log_log(logger, fawkes::Logger::LL_WARN));
	log_log_error_.reset(new log_log(logger, fawkes::Logger::LL_ERROR));
	xmlrpc_registry_->addMethod("log.entries", &*log_entries_);
	xmlrpc_registry_->addMethod("log.get_size", &*log_get_size_);
	xmlrpc_registry_->addMethod("log.set_size", &*log_set_size_);
	xmlrpc_registry_->addMethod("log.log_debug", &*log_log_debug_);
	xmlrpc_registry_->addMethod("log.log_info", &*log_log_info_);
	xmlrpc_registry_->addMethod("log.log_warn", &*log_log_warn_);
	xmlrpc_registry_->addMethod("log.log_error", &*log_log_error_);
}

/** Destructor. */
XmlRpcLogMethods::~XmlRpcLogMethods()
{
}

/** @class XmlRpcLogMethods::log_entries "log.h"
 * Get most recent log entries via XML-RPC method.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cache_logger cache logger to access recent log messages
 */
XmlRpcLogMethods::log_entries::log_entries(fawkes::CacheLogger *cache_logger)
{
	_signature = "A:";
	_help      = "Returns array of recent log messages. Each entry is a struct "
	        "consisting of the entries component, time string and message.";

	cache_logger_ = cache_logger;
}

/** Virtual empty destructor. */
XmlRpcLogMethods::log_entries::~log_entries()
{
}

/** Execute method.
 * @param params parameters
 * @param result result value
 */
void
XmlRpcLogMethods::log_entries::execute(xmlrpc_c::paramList const &params,
                                       xmlrpc_c::value *const     result)
{
	// No reference, copy!
	cache_logger_->lock();
	std::list<CacheLogger::CacheEntry> messages = cache_logger_->get_messages();
	cache_logger_->unlock();
	std::list<CacheLogger::CacheEntry>::iterator i;

	std::vector<xmlrpc_c::value> array;

	for (i = messages.begin(); i != messages.end(); ++i) {
		std::map<std::string, xmlrpc_c::value> elem;
		elem.insert(std::make_pair("component", xmlrpc_c::value_string(i->component)));
		elem.insert(std::make_pair("time", xmlrpc_c::value_datetime(i->time)));
		elem.insert(std::make_pair("message", xmlrpc_c::value_string(i->message)));
		array.push_back(xmlrpc_c::value_struct(elem));
	}

	*result = xmlrpc_c::value_array(array);
}

/** @class XmlRpcLogMethods::log_get_size "log.h"
 * XML-RPC method to get the current cache log size.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cache_logger cache logger to access recent log messages
 */
XmlRpcLogMethods::log_get_size::log_get_size(fawkes::CacheLogger *cache_logger)
{
	_signature = "i:";
	_help      = "Get current maximum size of the cache log.";

	cache_logger_ = cache_logger;
}

/** Virtual empty destructor. */
XmlRpcLogMethods::log_get_size::~log_get_size()
{
}

/** Execute method.
 * @param params parameters
 * @param result result value
 */
void
XmlRpcLogMethods::log_get_size::execute(xmlrpc_c::paramList const &params,
                                        xmlrpc_c::value *const     result)
{
	*result = xmlrpc_c::value_int(cache_logger_->size());
}

/** @class XmlRpcLogMethods::log_set_size "log.h"
 * XML-RPC method to set maximum size of cache logger.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cache_logger cache logger
 */
XmlRpcLogMethods::log_set_size::log_set_size(fawkes::CacheLogger *cache_logger)
{
	_signature = "n:i";
	_help      = "Set maximum size of cache logger.";

	cache_logger_ = cache_logger;
}

/** Virtual empty destructor. */
XmlRpcLogMethods::log_set_size::~log_set_size()
{
}

/** Execute method.
 * @param params parameters
 * @param result result value
 */
void
XmlRpcLogMethods::log_set_size::execute(xmlrpc_c::paramList const &params,
                                        xmlrpc_c::value *const     result)
{
	int new_size = params.getInt(0);
	if (new_size <= 0) {
		throw xmlrpc_c::fault("Illegal size value, must be integer > 0",
		                      xmlrpc_c::fault::CODE_UNSPECIFIED);
	}
	cache_logger_->set_size(new_size);
	*result = xmlrpc_c::value_nil();
}

/** @class XmlRpcLogMethods::log_log "log.h"
 * XML-RPC method to log a message.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger to output messages
 * @param log_level level to log messages at
 */
XmlRpcLogMethods::log_log::log_log(fawkes::Logger *logger, fawkes::Logger::LogLevel log_level)
{
	_signature = "n:ss";
	_help      = "Log message of specified level, arguments are component and message.";

	logger_    = logger;
	log_level_ = log_level;
}

/** Virtual empty destructor. */
XmlRpcLogMethods::log_log::~log_log()
{
}

/** Execute method.
 * @param params parameters
 * @param result result value
 */
void
XmlRpcLogMethods::log_log::execute(xmlrpc_c::paramList const &params, xmlrpc_c::value *const result)
{
	std::string component = params.getString(0);
	std::string message   = params.getString(1);
	logger_->log(log_level_, component.c_str(), "%s", message.c_str());
	*result = xmlrpc_c::value_nil();
}
