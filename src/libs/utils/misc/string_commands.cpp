
/***************************************************************************
 *  string.cpp - string utilities for command argv and envs
 *
 *  Created: Fri Aug 22 15:32:47 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include "string.h"

#include <list>
#include <tuple>
#include <string>
#include <cstring>
#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** Convert command args to string.
 * @param argv arguments, assumed to be standard args as passed to programs,
 * i.e. the first element is the executable, the following are the parameters.
 * @return string, where all elements of argv have been concatenated
 */
std::string
command_args_tostring(const char *argv[])
{
  std::string command = "";
  for (int i = 0; argv[i]; ++i) {
    if (i > 0)  command += " ";
    command += argv[i];
  }
  return command;
}


/** Convert environment to string.
 * This simply creates a string with semi-colon separated environment elements.
 * This is mostly useful for debug output of the environment.
 * @param envp environment string array
 * @return string with printable environment
 */
std::string
envp_tostring(char *envp[])
{
  std::string environment = "";
  for (int i = 0; envp[i]; ++i) {
    if (i > 0)  environment += "; ";
    environment += envp[i];
  }
  return environment;
}


/** Copy an environment and extend certain paths.
 * This will create a vector which comprises the environment in @p environ.
 * The path_ext are assumed to be pairwise entries of environment variable
 * name followed by an entry for the path extensions. Paths are here
 * colon-separated strings of paths, e.g. like the PATH environment variable.
 * If the variable had already been set, the given paths are appended to
 * the variable (a closing colon will be maintained if it exists). If they
 * were not set before, the entry is added.
 * @param environ environment to copy
 * @param path_ext path extension, an array of an odd number of elements,
 * always pairwise an entry for the variable name followed by the path extension.
 * The last element must always be NULL.
 * @return vector of strings with copied and extended environment
 */
std::vector<std::string>
envp_copy_expand(char *environ[], const char *path_ext[])
{
  std::list<std::tuple<std::string, std::string, bool>> path_ext_m;
  for (size_t i = 0; path_ext[i] && path_ext[i+1]; i += 2) {
    std::string match = std::string(path_ext[i]) + "=";
    path_ext_m.push_back(std::make_tuple(match, std::string(path_ext[i+1]), false));
  }

  unsigned int extra_ent = 0;

  size_t environ_length = 0;
  for (size_t i = 0; environ[i]; ++i) {
    ++environ_length;
    std::string ev = environ[i];
    for (auto &m : path_ext_m) {
      if (ev.find(std::get<0>(m)) == 0) {
	std::get<2>(m) = true;
	++extra_ent;
	break;
      }
    }
  }

  size_t envp_length = environ_length + extra_ent;
  std::vector<std::string> envp(envp_length);
  for (size_t i = 0; environ[i]; ++i) {
    std::string ev(environ[i]);
    for (auto m : path_ext_m) {
      if (ev.find(std::get<0>(m)) == 0) {
	// modify
	if (ev[ev.length()-1] == ':') {
	  ev += std::get<1>(m) + ":";
	} else {
	  ev += ":" + std::get<1>(m);
	}
      }
    }
    envp[i] = ev;
  }

  unsigned int extra_ind = 0;
  for (auto m : path_ext_m) {
    if (! std::get<2>(m)) {
      std::string ev = std::get<0>(m) + std::get<1>(m) + ":";
      envp[envp_length - extra_ent + extra_ind++] = ev;
    }
  }

  return envp;
}

} // end namespace fawkes


