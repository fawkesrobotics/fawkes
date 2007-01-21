
/***************************************************************************
 *  logger.cpp - Fawkes logging interface
 *
 *  Created: Tue Jan 16 20:40:15 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <utils/logging/logger.h>


/** @class Logger logging/logger.h
 * Interface for logging.
 * This interface facilitates a way to collect all output, be it debugging
 * output, informational output, warning or error messages.
 *
 * There should be no need no more for usage of printf in the code but
 * rather a logger should be used instead.
 *
 * The LoggingAspect should be added to a Thread that has to log something
 * (which is likely to be the case).
 *
 * A special note to logging hackers: A logger may never ever bounce. This
 * means that not printing a message is ok in case of an internal error in
 * the logger, but it may never indicate that error with an exception!
 * If a logger cannot deliver the messages as it should be (like a network
 * logger that cannot send because the connection is dead) it should at
 * least dump it to stderr!
 *
 * @fn void Logger::log_debug(const char *component, const char *format, ...) = 0
 * Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_info(const char *component, const char *format, ...) = 0
 * Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_warn(const char *component, const char *format, ...) = 0
 * Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_error(const char *component, const char *format, ...) = 0
 * Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_debug(const char *component, Exception &e) = 0
 * Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_info(const char *component, Exception &e) = 0
 * Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_warn(const char *component, Exception &e) = 0
 * Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_error(const char *component, Exception &e) = 0
 * Log error message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 */

/** Virtual empty destructor. */
Logger::~Logger()
{
}
