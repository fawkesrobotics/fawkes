
/***************************************************************************
 *  file.cpp - Fawkes file logger
 *
 *  Created: Tue Jan 16 16:56:49 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
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

#include <logging/file.h>
#include <utils/system/file.h>

#include <core/threading/mutex.h>

#include <cstdlib>
#include <sys/time.h>
#include <sys/stat.h>
#include <time.h>
#include <fcntl.h>
#include <cerrno>
#include <stdio.h>
#include <unistd.h>

#include <string>

namespace fawkes {

/** @class FileLogger <logging/file.h>
 * Interface for logging to a specified file.
 * The FileLogger will pipe all output into the given file. The
 * output will be prepended by a single character which determines the 
 * type of output (E for error, W for warning, etc.).
 *
 */

/** Constructor. 
 * The filename is generated from the filename pattern by replacing '$time' with
 * the current time.
 * @param filename_pattern the name pattern of the log-file
 * @param log_level minimum log level
 */
FileLogger::FileLogger(const char* filename_pattern, LogLevel log_level)
  : Logger(log_level)
{
  now_s = (struct tm *)malloc(sizeof(struct tm));
  struct timeval now;
  gettimeofday(&now, NULL);
  localtime_r(&now.tv_sec, now_s);
  char *start_time;
  if (asprintf(&start_time, "%04d-%02d-%02d_%02d-%02d-%02d",
    1900 + now_s->tm_year, now_s->tm_mon + 1, now_s->tm_mday, now_s->tm_hour,
    now_s->tm_min, now_s->tm_sec) == -1) {
    throw Exception("Failed to print current time");
  }
  std::string pattern(filename_pattern);
  std::string time_var = "$time";
  size_t pos = pattern.find(time_var);
  if (pos != std::string::npos) {
    pattern.replace(pos, time_var.length(), std::string(start_time));
  }
  free(start_time);
  const char *filename = pattern.c_str();
  int fd = open(filename, O_RDWR | O_CREAT | O_APPEND,
		S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
  if (fd == -1) {
    throw Exception(errno, "Failed to open log file %s", filename);
  }
  log_file = fdopen(fd, "a");
  // make buffer line-buffered
  setvbuf(log_file, NULL, _IOLBF, 0);

  // create a symlink for the latest log if the filename has a time stamp
  if (pos != std::string::npos) {
    std::string latest_filename(filename_pattern);
    latest_filename.replace(pos, time_var.length(), "latest");
    int link_res = symlink(filename, latest_filename.c_str());
    if (link_res == -1) {
      if (errno == EEXIST) {
        int unlink_res = unlink(latest_filename.c_str());
        if (unlink_res == -1) {
          throw Exception(errno, "Failed to update symlink at %s",
              latest_filename.c_str());
        }
        link_res = symlink(filename, latest_filename.c_str());
        if (link_res == -1) {
          throw Exception(errno, "Failed ot create symlink from %s to %s",
              filename, latest_filename.c_str());
        }
      } else {
          throw Exception(errno, "Failed ot create symlink from %s to %s",
              filename, latest_filename.c_str());
      }
    }
  }

  mutex = new Mutex();
}


/** Destructor. */
FileLogger::~FileLogger()
{
  free(now_s);
  fclose(log_file);
  delete mutex;
}


void
FileLogger::log_debug(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_debug(component, format, arg);
  va_end(arg);
}


void
FileLogger::log_info(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_info(component, format, arg);
  va_end(arg);
}


void
FileLogger::log_warn(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_warn(component, format, arg);
  va_end(arg);
}


void
FileLogger::log_error(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_error(component, format, arg);
  va_end(arg);
}


void
FileLogger::log_debug(const char *component, Exception &e)
{
  if ( log_level <= LL_DEBUG ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s [EXCEPTION]: ", "D", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
      fprintf(log_file, "%s", *i);
      fprintf(log_file, "\n");
    }
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::log_info(const char *component, Exception &e)
{
  if ( log_level <= LL_INFO ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s [EXCEPTION]: ", "I", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
      fprintf(log_file, "%s", *i);
      fprintf(log_file, "\n");
    }
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::log_warn(const char *component, Exception &e)
{
  if ( log_level <= LL_WARN ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s [EXCEPTION]: ", "W", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
      fprintf(log_file, "%s", *i);
      fprintf(log_file, "\n");
    }
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::log_error(const char *component, Exception &e)
{
  if ( log_level <= LL_ERROR ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s [EXCEPTION]: ", "E", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
      fprintf(log_file, "%s", *i);
      fprintf(log_file, "\n");
    }
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::vlog_debug(const char* component, const char* format, va_list va)
{
  if (log_level <= LL_DEBUG ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s: ", "D", now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
    vfprintf(log_file, format, va);
    fprintf(log_file, "\n");
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::vlog_info(const char *component, const char *format, va_list va)
{
  if (log_level <= LL_INFO ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s: ", "I", now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
    vfprintf(log_file, format, va);
    fprintf(log_file, "\n");
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  if (log_level <= LL_WARN ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s: ", "W", now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
    vfprintf(log_file, format, va);
    fprintf(log_file, "\n");
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::vlog_error(const char *component, const char *format, va_list va)
{
  if (log_level <= LL_ERROR ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s: ", "E", now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
    vfprintf(log_file, format, va);
    fprintf(log_file, "\n");
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::tlog_debug(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_debug(t, component, format, arg);
  va_end(arg);
}


void
FileLogger::tlog_info(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_info(t, component, format, arg);
  va_end(arg);
}


void
FileLogger::tlog_warn(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_warn(t, component, format, arg);
  va_end(arg);
}


void
FileLogger::tlog_error(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_error(t, component, format, arg);
  va_end(arg);
}


void
FileLogger::tlog_debug(struct timeval *t, const char *component, Exception &e)
{
  if ( log_level <= LL_DEBUG ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s [EXCEPTION]: ", "D", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
      fprintf(log_file, "%s", *i);
      fprintf(log_file, "\n");
    }
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::tlog_info(struct timeval *t, const char *component, Exception &e)
{
  if ( log_level <= LL_INFO ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s [EXCEPTION]: ", "I", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
      fprintf(log_file, "%s", *i);
      fprintf(log_file, "\n");
    }
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::tlog_warn(struct timeval *t, const char *component, Exception &e)
{
  if ( log_level <= LL_WARN ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s [EXCEPTION]: ", "W", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
      fprintf(log_file, "%s", *i);
      fprintf(log_file, "\n");
    }
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::tlog_error(struct timeval *t, const char *component, Exception &e)
{
  if ( log_level <= LL_ERROR ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s [EXCEPTION]: ", "E", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
      fprintf(log_file, "%s", *i);
      fprintf(log_file, "\n");
    }
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::vtlog_debug(struct timeval *t, const char* component, const char* format, va_list va)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s: ", "D", now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
    vfprintf(log_file, format, va);
    fprintf(log_file, "\n");
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::vtlog_info(struct timeval *t, const char *component, const char *format, va_list va)
{
  if (log_level <= LL_INFO ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s: ", "I", now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
    vfprintf(log_file, format, va);
    fprintf(log_file, "\n");
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::vtlog_warn(struct timeval *t, const char *component, const char *format, va_list va)
{
  if (log_level <= LL_WARN ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s: ", "W", now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
    vfprintf(log_file, format, va);
    fprintf(log_file, "\n");
    fflush(log_file);
    mutex->unlock();
  }
}


void
FileLogger::vtlog_error(struct timeval *t, const char *component, const char *format, va_list va)
{
  if (log_level <= LL_ERROR ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    fprintf(log_file, "%s %02d:%02d:%02d.%06ld %s: ", "E", now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
    vfprintf(log_file, format, va);
    fprintf(log_file, "\n");
    fflush(log_file);
    mutex->unlock();
  }
}


} // end namespace fawkes
