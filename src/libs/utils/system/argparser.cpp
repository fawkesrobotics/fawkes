
/***************************************************************************
 *  argparser.cpp - Implementation of the argument parser
 *
 *  Generated: Mon May 30 13:25:33 2005 (from FireVision)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/argparser.h>
#include <core/exceptions/software.h>
#include <libgen.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace fawkes {

/** @class ArgumentParser <utils/system/argparser.h>
 * Parse command line arguments.
 * Interface to GNU getopt and getopt_long. Parses command line arguments and
 * separates long and short options.
 *
 * The supplied opt_string is a string containing the legitimate option
 * characters. A character c denotes an option of the type "-c" (single dash).
 * If such a character is followed by a colon, the option requires an argument,
 * Two colons mean an option takes an optional arg.
 *
 * If long_options is supplied options started out by two dashes are recognized.
 * Long option names may be abbreviated if the abbreviation is unique or is an
 * exact match for some defined option. A long option may take a parameter, of
 * the form --arg=param or --arg param.
 *
 * long_options is a pointer to the first element of an array of struct option
 * declared in <getopt.h> as
 *
 * @code
 * struct option {
 *   const char *name;
 *   int has_arg;
 *   int *flag;
 *   int val;
 * };
 * @endcode
 *
 * The meanings of the different fields are:
 *
 * name   is the name of the long option.
 *
 *  has_arg   is: no_argument (or 0) if the option does not take  an  argument;
 *            required_argument  (or  1)  if  the  option  requires  an  argument;
 *            or optional_argument (or 2) if the option takes an optional argument.
 *
 *     flag   specifies  how results are returned for a long option.  If flag is
 *            NULL, then getopt_long() returns val.  (For example, the calling
 *            program may set val to the equivalent short option character.)
 *            Otherwise, getopt_long() returns 0, and flag points to a variable
 *            which is set to val if the option is found, but left unchanged if the
 *            option is not found. Handled internally in ArgumentParser
 *
 * For more information see man 3 getopt.
 *
 * All arguments that do not belong to parsed options are stored as items and can
 * be retrieved via items().
 */


/** Constructor
 * @param argc argument count.
 * @param argv argument vector
 * @param opt_string option string, see ArgumentParser
 * @param long_options long options, see ArgumentParser
 */
ArgumentParser::ArgumentParser(int argc, char **argv,
			       const char *opt_string, option *long_options)
{
  __argc = argc;
  __argv = argv;

  __opt_string = opt_string;

  if (long_options) {
    option *tmplo = long_options;
    while (tmplo->name != 0) {
      __long_opts.push_back(*tmplo);
      tmplo += 1;
    }
  }

  __opts.clear();
  __items.clear();

#ifdef _GNU_SOURCE
  __program_name = strdup(basename( argv[0] ));
#else
  // Non-GNU variants may modify the sting in place
  char *tmp = strdup(argv[0]);
  __program_name = strdup(basename(tmp));
  free(tmp);
#endif

  if (long_options == NULL) {
    int c ;
    char tmp[2];

    while ((c = getopt(argc, argv, opt_string)) != -1) {
      if (c == '?') {
	throw UnknownArgumentException(c);
      } else if (c == ':') {
	throw MissingArgumentException(c);
      }
      sprintf(tmp, "%c", c);
      __opts[ tmp ] = optarg;
    }
  } else {
    int opt_ind = 0;
    int c;
    while ((c = getopt_long(argc, argv, opt_string, long_options, &opt_ind)) != -1) {
      if (c == '?') {
	throw UnknownArgumentException(c);
      } else if (c == 0) {
	// long options
	__opts[ long_options[opt_ind].name ] = optarg;
      } else {
	char tmp[2];
	sprintf(tmp, "%c", c);
	__opts[ tmp ] = optarg;
      }
    }
  }

  __items.clear();
  int ind = optind;
  while (ind < argc) {
    __items.push_back( argv[ind++] );
  }

}


/** Destructor. */
ArgumentParser::~ArgumentParser()
{
  free(__program_name);
  __opts.clear();
}


/** Check if argument has been supplied.
 * @param argn argument name to check for
 * @return true, if the argument was given on the command line, false otherwise
 */
bool
ArgumentParser::has_arg(const char *argn)
{
  return (__opts.count((char *)argn) > 0);
}


/** Get argument value.
 * Use this method to get the value supplied to the given option.
 * @param argn argument name to retrieve
 * @return the argument value. Pointer to static program array. Do not free!
 * Returns NULL if argument was not supplied on command line.
 */
const char *
ArgumentParser::arg(const char *argn)
{
  if ((__opts.count(argn) > 0) && (__opts[argn] != NULL)) {
    return __opts[ (char *)argn ];
  } else {
    return NULL;
  }
}


/** Get argument while checking availability.
 * The argument will be a newly allocated copy of the string. You have to
 * free it after you are done with it.
 * @param argn argument name to retrieve
 * @param value a pointer to a newly allocated copy of the argument value will
 * be stored here if the argument has been found.
 * The value is unchanged if argument was not supplied.
 * @return true, if the argument was supplied, false otherwise
 */
bool
ArgumentParser::arg(const char *argn, char **value)
{
  if ((__opts.count(argn) > 0) && (__opts[argn] != NULL)) {
    *value = strdup(__opts[ (char *)argn ]);
    return true;
  } else {
    return false;
  }
}


/** Parse host:port string.
 * The value referenced by the given argn is parsed for the pattern "host:port".
 * If the string does not match this pattern an exception is thrown.
 * The host will be a newly allocated copy of the string. You have to
 * free it after you are done with it. If no port is supplied in the string (plain
 * hostname string) the port argument is left unchanged. If the argument has not
 * been supplied at all both values are left unchanged. Thus it is safe to put the
 * default values into the variables before passing them to this method. Note
 * however that you have to free the returned host string in case of a successful
 * return, and only in that case probably!
 * @param argn argument name to retrieve
 * @param host Upon successful return contains a pointer to a newly alloated string
 * with the hostname part. Free it after you are finished.
 * @param port upon successful return contains the port part
 * @return true, if the argument was supplied, false otherwise
 * @exception OutOfBoundsException thrown if port is not in the range [0..65535]
 */
bool
ArgumentParser::parse_hostport(const char *argn, char **host,
			       unsigned short int *port)
{
  if ((__opts.count(argn) > 0) && (__opts[argn] != NULL)) {
    char *tmpvalue = strdup(__opts[ (char *)argn ]);

    if ( strchr(tmpvalue, ':') != NULL ) {
      char *save_ptr;
      *host = strtok_r(tmpvalue, ":", &save_ptr);
      char *tmpport = strtok_r(NULL, "", &save_ptr);

      int port_num = atoi(tmpport);
      if ( (port_num < 0) || (port_num > 0xFFFF) ) {
	throw OutOfBoundsException("Invalid port", port_num, 0, 0xFFFF);
      }
      *port = port_num;
    } else {
      *host = tmpvalue;
    }

    return true;
  } else {
    return false;
  }
}


/** Parse host:port string.
 * The value referenced by the given argn is parsed for the pattern "host:port". If the
 * string does not match this pattern an exception is thrown.
 * If no port is supplied in the string (plain
 * hostname string) the port argument is left unchanged. If the argument has not
 * been supplied at all both values are left unchanged. Thus it is safe to put the default
 * values into the variables before passing them to this method.
 * @param argn argument name to retrieve
 * @param host Upon successful return contains the hostname part
 * @param port upon successful return contains the port part (unchanged if not supplied)
 * @return true, if the argument was supplied, false otherwise
 * @exception OutOfBoundsException thrown if port is not in the range [0..65535]
 */
bool
ArgumentParser::parse_hostport(const char *argn, std::string &host, unsigned short int &port)
{
  if ((__opts.count(argn) == 0) || (__opts[argn] == NULL)) return false;

  std::string tmpvalue = __opts[argn];

  size_t col_idx = tmpvalue.find_last_of(':');
  if ( col_idx == tmpvalue.npos ) {
    host = tmpvalue;
  }
  else
  {
    host = tmpvalue.substr(0, col_idx);
    std::string tmpport = tmpvalue.substr(col_idx + 1);

    int port_num = atoi(tmpport.c_str());
    if ( (port_num < 0) || (port_num > 0xFFFF) ) {
      throw OutOfBoundsException("Invalid port", port_num, 0, 0xFFFF);
    }
    port = port_num;
  }
  return true;
}


/** Parse argument as integer.
 * Converts the value of the given argument to an integer.
 * @param argn argument name to retrieve
 * @return value of string as long int
 * @exception IllegalArgumentException thrown if the value cannot be properly
 * converted to an integer
 * @exception Exception thrown if the argument has not been supplied
 */
long int
ArgumentParser::parse_int(const char *argn)
{
  if ((__opts.count(argn) > 0) && (__opts[argn] != NULL)) {
    char *endptr;
    long int rv = strtol(__opts[argn], &endptr, 10);
    if ( endptr[0] != 0 ) {
      throw IllegalArgumentException("Supplied argument is not of type int");
    }
    return rv;
  } else {
    throw Exception("Value for '%s' not available", argn);
  }
}


/** Parse argument as double.
 * Converts the value of the given argument to a double.
 * @param argn argument name to retrieve
 * @return value of string as double
 * @exception IllegalArgumentException thrown if the value cannot be properly
 * converted to a double
 * @exception Exception thrown if the argument has not been supplied
 */
double
ArgumentParser::parse_float(const char *argn)
{
  if ((__opts.count(argn) > 0) && (__opts[argn] != NULL)) {
    char *endptr;
    double rv = strtod(__opts[argn], &endptr);
    if ( endptr[0] != 0 ) {
      throw IllegalArgumentException("Supplied argument is not of type double");
    }
    return rv;
  } else {
    throw Exception("Value for '%s' not available", argn);
  }
}


/** Parse item as integer.
 * Converts the value of the given item to an integer.
 * @param index item index
 * @return value of string as long int
 * @exception IllegalArgumentException thrown if the value cannot be properly
 * converted to an integer
 * @exception Exception thrown if the argument has not been supplied
 */
long int
ArgumentParser::parse_item_int(unsigned int index)
{
  if (index < __items.size()) {
    char *endptr;
    long int rv = strtol(__items[index], &endptr, 10);
    if ( endptr[0] != 0 ) {
      throw IllegalArgumentException("Supplied argument is not of type int");
    }
    return rv;
  } else {
    throw Exception("Value for item %u not available", index);
  }
}


/** Parse item as double.
 * Converts the value of the given item to a double.
 * @param index item index
 * @return value of string as double
 * @exception IllegalArgumentException thrown if the value cannot be properly
 * converted to a double
 * @exception Exception thrown if the argument has not been supplied
 */
double
ArgumentParser::parse_item_float(unsigned int index)
{
  if (index < __items.size()) {
    char *endptr;
    double rv = strtod(__items[index], &endptr);
    if ( endptr[0] != 0 ) {
      throw IllegalArgumentException("Supplied argument is not of type double");
    }
    return rv;
  } else {
    throw Exception("Value for item %u not available", index);
  }
}


/** Get non-option items.
 * @return pointer to vector of pointer to non-argument values. Handled internally,
 * do not free or delete!
 */
const std::vector< const char* > &
ArgumentParser::items() const
{
  return __items;
}


/** Get number of non-option items.
 * @return number of non-opt items.
 */
std::vector< const char* >::size_type
ArgumentParser::num_items() const
{
  return __items.size();
}


/** Get number of arguments.
 * @return number of arguments
 */
int
ArgumentParser::argc() const
{
  return __argc;
}


/** Program argument array as supplied to constructor.
 * @return argument array.
 */
const char **
ArgumentParser::argv() const
{
  return (const char **)__argv;
}


/** Get name of program.
 * @return the name of the program (argv[0] of argument vector supplied to constructor).
 */
const char *
ArgumentParser::program_name() const
{
  return __program_name;
}


} // end namespace fawkes
