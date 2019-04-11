
/***************************************************************************
 *  mongodb_logger_thread.cpp - MongoDB logger thread
 *
 *  Created: Tue Dec 07 22:59:47 2010
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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

#include <bsoncxx/builder/basic/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/operation_exception.hpp>

using namespace mongocxx;
using namespace fawkes;

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
	if (log_level <= ll) {
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

		free(msg);

		try {
			mongodb_client->database(database_)[collection_].insert_one(b.view());
		} catch (operation_exception &e) {
		} // ignored
	}
}

void
MongoLogLoggerThread::insert_message(LogLevel ll, const char *component, Exception &e)
{
	if (log_level <= ll) {
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
	if (log_level <= ll) {
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
}

void
MongoLogLoggerThread::tlog_insert_message(LogLevel        ll,
                                          struct timeval *t,
                                          const char *    component,
                                          Exception &     e)
{
	if (log_level <= ll) {
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
