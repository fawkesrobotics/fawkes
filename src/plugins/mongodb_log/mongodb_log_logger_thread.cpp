
/***************************************************************************
 *  mongodb_logger_thread.cpp - MongoDB logger thread
 *
 *  Created: Tue Dec 07 22:59:47 2010
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
 *             2020       Daniel Swoboda [swoboda@kbsg.rwth-aachen.de]
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

#include "mongodb_log_logger_thread.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <bsoncxx/json.hpp>
#include <iostream>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/operation_exception.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/options/find.hpp>
#include <mongocxx/uri.hpp>

using namespace mongocxx;
using namespace fawkes;

using bsoncxx::builder::basic::kvp;
using bsoncxx::builder::basic::make_document;

/** @class MongoLogLoggerThread "mongodb_log_logger_thread.h"
 * Thread that provides a logger writing to MongoDB.
 * This thread provides a logger, which writes log information to a
 * MongoDB collection.
 * @author Tim Niemueller
 */

/** Constructor. */
MongoLogLoggerThread::MongoLogLoggerThread()
: Thread("MongoLogLoggerThread", Thread::OPMODE_WAITFORWAKEUP),
  LoggerAspect(this),
  MongoDBAspect("default")
{
	mutex_ = new Mutex();
}

/** Destructor. */
MongoLogLoggerThread::~MongoLogLoggerThread()
{
	delete mutex_;
}

void
MongoLogLoggerThread::init()
{
	database_   = config->get_string_or_default("/plugins/mongodb/logger/database", "fawkes");
	collection_ = config->get_string_or_default("/plugins/mongodb/logger/collection", "msglog");
	mongocxx::uri uri("mongodb://localhost:27017");
	mongodb_client = new mongocxx::client(uri);
}

void
MongoLogLoggerThread::finalize()
{
}

void
MongoLogLoggerThread::loop()
{
}

void
MongoLogLoggerThread::insert_message(LogLevel    ll,
                                     const char *component,
                                     const char *format,
                                     va_list     va)
{
	MutexLocker            lock(mutex_);
	bsoncxx::types::b_date nowd{std::chrono::high_resolution_clock::now()};

	char *msg;
	if (vasprintf(&msg, format, va) == -1) {
		// Cannot do anything useful, drop log message
		return;
	}

	using namespace bsoncxx::builder;
	basic::document b;
	switch (ll) {
	case LL_DEBUG: b.append(basic::kvp("level", "DEBUG")); break;
	case LL_INFO: b.append(basic::kvp("level", "INFO")); break;
	case LL_WARN: b.append(basic::kvp("level", "WARN")); break;
	case LL_ERROR: b.append(basic::kvp("level", "ERROR")); break;
	default: b.append(basic::kvp("level", "UNKN")); break;
	}

	b.append(basic::kvp("component", component));
	b.append(basic::kvp("time", nowd));
	b.append(basic::kvp("message", msg));
	b.append(basic::kvp("game", gametime_));

	try {
		mongodb_client->database(database_)[collection_].insert_one(b.view());
	} catch (operation_exception &e) {
	} // ignored

	std::string msg_s(msg);

	//track assertion
	if (msg_s.find("(wm-fact (id \"/domain/") != std::string::npos
	    && msg_s.find("==>") != std::string::npos) {
		basic::document df;
		basic::document dfc;
		df.append(
		  basic::kvp("id",
		             msg_s.substr(msg_s.find("(id \"") + 5,
		                          msg_s.find("\")") - msg_s.find("(id \"") - 5)));
		df.append(basic::kvp(
		  "is-list",
		  msg_s.substr(msg_s.find("(is-list ") + 9)
		    .substr(0, msg_s.substr(msg_s.find("(is-list ") + 9).find(")")))); 
		df.append(basic::kvp("source",
		                     config->get_string_or_default("fawkes/agent/name", "UNKN"))); 
		df.append(
		  basic::kvp("type",
		             msg_s.substr(msg_s.find("(type ") + 6)
		               .substr(0,
		                       msg_s.substr(msg_s.find("(type ") + 6).find(")")))); 
		if (msg_s.substr(msg_s.find("(is-list ") + 9)
		      .substr(0, msg_s.substr(msg_s.find("(is-list ") + 9).find(")"))
		    == "TRUE") {
			auto array_builder = basic::array{};

			std::string values_string = msg_s.substr(msg_s.find("(values") + 8)
			                              .substr(0, msg_s.substr(msg_s.find("(values") + 8).find(")"));

			size_t      pos = 0;
			std::string token;
			while ((pos = values_string.find(" ")) != std::string::npos) {
				token = values_string.substr(0, pos);
				array_builder.append(token);
				values_string.erase(0, pos + 1);
			}
			df.append(basic::kvp("values", array_builder.extract()));
		} else {
			df.append(basic::kvp("value",
			                     msg_s.substr(msg_s.find("(value ") + 7)
			                       .substr(0, msg_s.substr(msg_s.find("(value ") + 7).find(")"))));
		}
		df.append(basic::kvp("update-timestamp", nowd));

		dfc.append(basic::kvp("game", gametime_));
		dfc.append(basic::kvp("asserted", nowd));
		dfc.append(basic::kvp("retracted", "FALSE"));
		dfc.append(basic::kvp("fact", df));
		dfc.append(basic::kvp("msg", msg_s));
		dfc.append(
		  basic::kvp("clips-id", msg_s.substr(msg_s.find("==> ") + 4).substr(0, msg_s.find("(") - 5)));
		mongodb_client->database(database_)["gamestate_recovery_test"].insert_one(dfc.view());
	}

	//track retraction
	if (msg_s.find("(wm-fact (id \"/domain/") != std::string::npos
	    && msg_s.find("<==") != std::string::npos) {
		std::string clips_id = msg_s.substr(msg_s.find("<== ") + 4).substr(0, msg_s.find("(") - 5);

		mongodb_client->database(database_)["gamestate_recovery_test"].update_one(
		  make_document(kvp("clips-id", clips_id)),
		  make_document(kvp("$set", make_document(kvp("retracted", nowd)))));
	}
	free(msg);
}

void
MongoLogLoggerThread::insert_message(LogLevel ll, const char *component, Exception &e)
{
	MutexLocker            lock(mutex_);
	bsoncxx::types::b_date nowd{std::chrono::high_resolution_clock::now()};

	for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
		using namespace bsoncxx::builder;
		basic::document b;
		switch (ll) {
		case LL_DEBUG: b.append(basic::kvp("level", "DEBUG")); break;
		case LL_INFO: b.append(basic::kvp("level", "INFO")); break;
		case LL_WARN: b.append(basic::kvp("level", "WARN")); break;
		case LL_ERROR: b.append(basic::kvp("level", "ERROR")); break;
		default: b.append(basic::kvp("level", "UNKN")); break;
		}
		b.append(basic::kvp("component", component));
		b.append(basic::kvp("time", nowd));
		b.append(basic::kvp("message", std::string("[EXCEPTION] ") + *i));
		try {
			mongodb_client->database(database_)[collection_].insert_one(b.view());
		} catch (operation_exception &e) {
		} // ignored
	}
}

void
MongoLogLoggerThread::vlog_debug(const char *component, const char *format, va_list va)
{
	insert_message(LL_DEBUG, component, format, va);
}

void
MongoLogLoggerThread::vlog_info(const char *component, const char *format, va_list va)
{
	insert_message(LL_INFO, component, format, va);
}

void
MongoLogLoggerThread::vlog_warn(const char *component, const char *format, va_list va)
{
	insert_message(LL_WARN, component, format, va);
}

void
MongoLogLoggerThread::vlog_error(const char *component, const char *format, va_list va)
{
	insert_message(LL_ERROR, component, format, va);
}

void
MongoLogLoggerThread::log_debug(const char *component, const char *format, ...)
{
	va_list arg;
	va_start(arg, format);
	insert_message(LL_DEBUG, component, format, arg);
	va_end(arg);
}

void
MongoLogLoggerThread::log_info(const char *component, const char *format, ...)
{
	va_list arg;
	va_start(arg, format);
	insert_message(LL_INFO, component, format, arg);
	va_end(arg);
}

void
MongoLogLoggerThread::log_warn(const char *component, const char *format, ...)
{
	va_list arg;
	va_start(arg, format);
	insert_message(LL_WARN, component, format, arg);
	va_end(arg);
}

void
MongoLogLoggerThread::log_error(const char *component, const char *format, ...)
{
	va_list arg;
	va_start(arg, format);
	insert_message(LL_ERROR, component, format, arg);
	va_end(arg);
}

void
MongoLogLoggerThread::log_debug(const char *component, Exception &e)
{
	insert_message(LL_DEBUG, component, e);
}

void
MongoLogLoggerThread::log_info(const char *component, Exception &e)
{
	insert_message(LL_INFO, component, e);
}

void
MongoLogLoggerThread::log_warn(const char *component, Exception &e)
{
	insert_message(LL_WARN, component, e);
}

void
MongoLogLoggerThread::log_error(const char *component, Exception &e)
{
	insert_message(LL_ERROR, component, e);
}

void
MongoLogLoggerThread::tlog_insert_message(LogLevel        ll,
                                          struct timeval *t,
                                          const char *    component,
                                          const char *    format,
                                          va_list         va)
{
	MutexLocker lock(mutex_);
	char *      msg;
	if (vasprintf(&msg, format, va) == -1) {
		return;
	}

	bsoncxx::types::b_date nowd{
	  std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>{
	    std::chrono::milliseconds{t->tv_sec * 1000 + t->tv_usec / 1000}}};

	using namespace bsoncxx::builder;
	basic::document b;
	switch (ll) {
	case LL_DEBUG: b.append(basic::kvp("level", "DEBUG")); break;
	case LL_INFO: b.append(basic::kvp("level", "INFO")); break;
	case LL_WARN: b.append(basic::kvp("level", "WARN")); break;
	case LL_ERROR: b.append(basic::kvp("level", "ERROR")); break;
	default: b.append(basic::kvp("level", "UNKN")); break;
	}
	b.append(basic::kvp("component", component));
	b.append(basic::kvp("time", nowd));
	b.append(basic::kvp("message", msg));
	try {
		mongodb_client->database(database_)[collection_].insert_one(b.view());
	} catch (operation_exception &e) {
	} // ignored

	free(msg);

	mutex_->unlock();
}

void
MongoLogLoggerThread::tlog_insert_message(LogLevel        ll,
                                          struct timeval *t,
                                          const char *    component,
                                          Exception &     e)
{
	MutexLocker            lock(mutex_);
	bsoncxx::types::b_date nowd{
	  std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>{
	    std::chrono::milliseconds{t->tv_sec * 1000 + t->tv_usec / 1000}}};
	for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
		using namespace bsoncxx::builder;
		basic::document b;
		switch (ll) {
		case LL_DEBUG: b.append(basic::kvp("level", "DEBUG")); break;
		case LL_INFO: b.append(basic::kvp("level", "INFO")); break;
		case LL_WARN: b.append(basic::kvp("level", "WARN")); break;
		case LL_ERROR: b.append(basic::kvp("level", "ERROR")); break;
		default: b.append(basic::kvp("level", "UNKN")); break;
		}
		b.append(basic::kvp("component", component));
		b.append(basic::kvp("time", nowd));
		b.append(basic::kvp("message", std::string("[EXCEPTION] ") + *i));
		try {
			mongodb_client->database(database_)[collection_].insert_one(b.view());
		} catch (operation_exception &e) {
		} // ignored
	}
}

void
MongoLogLoggerThread::tlog_debug(struct timeval *t, const char *component, const char *format, ...)
{
	va_list arg;
	va_start(arg, format);
	tlog_insert_message(LL_DEBUG, t, component, format, arg);
	va_end(arg);
}

void
MongoLogLoggerThread::tlog_info(struct timeval *t, const char *component, const char *format, ...)
{
	va_list arg;
	va_start(arg, format);
	tlog_insert_message(LL_INFO, t, component, format, arg);
	va_end(arg);
}

void
MongoLogLoggerThread::tlog_warn(struct timeval *t, const char *component, const char *format, ...)
{
	va_list arg;
	va_start(arg, format);
	tlog_insert_message(LL_WARN, t, component, format, arg);
	va_end(arg);
}

void
MongoLogLoggerThread::tlog_error(struct timeval *t, const char *component, const char *format, ...)
{
	va_list arg;
	va_start(arg, format);
	tlog_insert_message(LL_ERROR, t, component, format, arg);
	va_end(arg);
}

void
MongoLogLoggerThread::tlog_debug(struct timeval *t, const char *component, Exception &e)
{
	tlog_insert_message(LL_DEBUG, t, component, e);
}

void
MongoLogLoggerThread::tlog_info(struct timeval *t, const char *component, Exception &e)
{
	tlog_insert_message(LL_INFO, t, component, e);
}

void
MongoLogLoggerThread::tlog_warn(struct timeval *t, const char *component, Exception &e)
{
	tlog_insert_message(LL_WARN, t, component, e);
}

void
MongoLogLoggerThread::tlog_error(struct timeval *t, const char *component, Exception &e)
{
	tlog_insert_message(LL_ERROR, t, component, e);
}

void
MongoLogLoggerThread::vtlog_debug(struct timeval *t,
                                  const char *    component,
                                  const char *    format,
                                  va_list         va)
{
	tlog_insert_message(LL_DEBUG, t, component, format, va);
}

void
MongoLogLoggerThread::vtlog_info(struct timeval *t,
                                 const char *    component,
                                 const char *    format,
                                 va_list         va)
{
	tlog_insert_message(LL_INFO, t, component, format, va);
}

void
MongoLogLoggerThread::vtlog_warn(struct timeval *t,
                                 const char *    component,
                                 const char *    format,
                                 va_list         va)
{
	tlog_insert_message(LL_WARN, t, component, format, va);
}

void
MongoLogLoggerThread::vtlog_error(struct timeval *t,
                                  const char *    component,
                                  const char *    format,
                                  va_list         va)
{
	tlog_insert_message(LL_ERROR, t, component, format, va);
}
