
/***************************************************************************
 *  argparser.h - Header for argument parser
 *
 *  Generated: Mon May 30 13:07:41 2005 (from FireVision)
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef _UTILS_SYSTEM_ARGPARSER_H_
#define _UTILS_SYSTEM_ARGPARSER_H_

#include <core/exception.h>
#include <utils/misc/string_compare.h>

#include <getopt.h>
#include <map>
#include <string>
#include <vector>

namespace fawkes {

/** Thrown if unknown argument was supplied */
class UnknownArgumentException : public Exception
{
public:
	/** Constructor.
   * @param c Unknown option character
   */
	UnknownArgumentException(char c) : Exception()
	{
		append("Unknown option '%c'", c);
	}
};

/** Thrown if required argument was missing. */
class MissingArgumentException : public Exception
{
public:
	/** Constructor.
   * @param c option with missing argument
   */
	MissingArgumentException(char c) : Exception()
	{
		append("Option '%c' requires an argument", c);
	}
};

class ArgumentParser
{
public:
	ArgumentParser(int argc, char **argv, const char *opt_string, option *long_options = NULL);
	~ArgumentParser();

	bool        has_arg(const char *argn);
	const char *arg(const char *argn);
	bool        arg(const char *argn, char **value);
	const char *program_name() const;

	bool parse_hostport(const char *argn, char **host, unsigned short int *port);
	bool parse_hostport(const char *argn, std::string &host, unsigned short int &port);

	static void parse_hostport_s(const char *s, char **host, unsigned short int *port);
	static void parse_hostport_s(const char *s, std::string &host, unsigned short int &port);

	long int parse_int(const char *argn);
	double   parse_float(const char *argn);

	long int parse_item_int(unsigned int index);
	double   parse_item_float(unsigned int index);

	const std::vector<const char *> &    items() const;
	std::vector<const char *>::size_type num_items() const;

	int          argc() const;
	const char **argv() const;

	/** Get option string.
   * @return option string used to create instance */
	std::string
	get_optstring() const
	{
		return opt_string_;
	}

	/** Get long option configuration.
   * @return vector of long options used to create instance */
	std::vector<option>
	get_long_opts() const
	{
		return long_opts_;
	}

private:
	std::map<std::string, const char *> opts_;
	std::map<std::string, const char *> opts_cit_;
	std::vector<const char *>           items_;

	char * program_name_;
	char **argv_;
	int    argc_;

	std::string         opt_string_;
	std::vector<option> long_opts_;
};

} // end namespace fawkes

#endif
