
/***************************************************************************
 *  argparser.cpp - Implementation of the argument parser
 *
 *  Generated: Mon May 30 13:25:33 2005 (from FireVision)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/argparser.h>

/** @class ArgumentParser utils/system/argparser.h
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
 * be retrieved via getItems().
 */


/** Constructor
 * @param argc argument count.
 * @param argv argument vector
 * @param opt_string option string, see ArgumentParser
 * @param long_options long options, see ArgumentParser
 */
ArgumentParser::ArgumentParser(int argc, char **argv, char *opt_string, option *long_options)
{
  this->argc = argc;
  this->argv = argv;

  program_name = basename( argv[0] );

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
      opts[ std::string(tmp) ] = optarg;
    }
  } else {
    int opt_ind = 0;
    int c;
    while ((c = getopt_long(argc, argv, opt_string, long_options, &opt_ind)) != -1) {
      if (c == '?') {
	throw UnknownArgumentException(c);
      } else if (c == 0) {
	// long options
	opts[ std::string(long_options[opt_ind].name) ] = optarg;
      } else {
	char tmp[2];
	sprintf(tmp, "%c", c);
	opts[ std::string(tmp) ] = optarg;
      }
    }
  }

  items.clear();
  int ind = optind;
  while (ind < argc) {
    items.push_back( argv[ind++] );
  }

}


/** Check if argument has been supplied.
 * @param arg argument to check for
 * @return true, if the argument was given on the command line, false otherwise
 */
bool
ArgumentParser::hasArgument(std::string arg)
{
  return (opts.count(arg) > 0);
}


/** Get argument value.
 * Use this method to get the value supplied to the given option.
 * @param arg argument to retrieve
 * @return the argument value. Pointer to static program array. Do not free!
 * Returns NULL if argument was not supplied on command line.
 */
char *
ArgumentParser::getArgument(std::string arg)
{
  if (opts.count(arg) > 0) {
    return opts[ arg ];
  } else {
    return NULL;
  }
}


/** Get argument while checking availability.
 * @param arg argument to retrieve
 * @param value a pointer to the argument value will be stored here if the argument
 * has been found. The value is unchanged if argument was not supplied.
 * @return true, if the argument was supplied, false otherwise
 */
bool
ArgumentParser::getArgument(std::string arg, char **value)
{
  if (opts.count(arg) > 0) {
    *value = opts[ arg ];
    return true;
  } else {
    return false;
  }
}


/** Get non-option items.
 * @return pointer to vector of pointer to non-argument values. Handled internally,
 * do not free or delete!
 */
std::vector< char* > *
ArgumentParser::getItems()
{
  return &items;
}


/** Get number of arguments.
 * @return number of arguments
 */
int
ArgumentParser::getArgC()
{
  return argc;
}


/** Program argument array as supplied to constructor.
 * @return argument array.
 */
char **
ArgumentParser::getArgV()
{
  return argv;
}


/** Get name of program.
 * @return the name of the program (argv[0] of argument vector supplied to constructor).
 */
std::string
ArgumentParser::getProgramName()
{
  return program_name;
}
