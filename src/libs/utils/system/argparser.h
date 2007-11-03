
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
#define __UTILS_SYSTEM_ARGPARSER_H_

#include <core/exception.h>

#include <utils/misc/string_compare.h>
#include <ext/hash_map>
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
  ~ArgumentParser();

  bool         has_arg(const char *argn);
  const char * arg(const char *argn);
  bool         arg(const char *argn, char **value);
  const char * program_name() const;

  const std::vector< const char * > &    items() const;
  std::vector< const char * >::size_type num_items() const;

  int            argc() const;
  const char **  argv() const;

 private:

  __gnu_cxx::hash_map< char *, const char *, __gnu_cxx::hash<char *>, StringEquality > _opts;
  __gnu_cxx::hash_map< char *, const char *, __gnu_cxx::hash<char *>, StringEquality >::const_iterator  _opts_cit;
  std::vector< const char * >  _items;

  char *  _program_name;
  char ** _argv;
  int     _argc;

};

#endif
