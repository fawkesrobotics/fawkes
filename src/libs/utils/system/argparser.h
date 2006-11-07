
/***************************************************************************
 *  argparser.h - Header for argument parser
 *
 *  Generated: Mon May 30 13:07:41 2005 (from FireVision)
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_SYSTEM_ARGPARSER_H_
#define __UTILS_SYSTME_ARGPARSER_H_

#include <core/exception.h>

#include <string>
#include <map>
#include <vector>

#include <getopt.h>

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
  ArgumentParser(int argc, char **argv, char *opt_string, option *long_options = NULL);

  bool hasArgument(std::string arg);
  char * getArgument(std::string arg);
  bool getArgument(std::string arg, char **value);
  std::string getProgramName();

  std::vector< char* > * getItems();

  int      getArgC();
  char **  getArgV();

 private:

  std::map< std::string, char * > opts;
  std::vector< char * > items;
  std::string program_name;

  char **argv;
  int    argc;

};

#endif
