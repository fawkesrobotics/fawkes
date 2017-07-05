
/***************************************************************************
 *  string.h - string utilities for command argv and envs
 *
 *  Created: Fri Aug 22 14:49:05 2014
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

#ifndef __PLUGINS_UTILS_STRING_H_
#define __PLUGINS_UTILS_STRING_H_

#include <string>
#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

std::string command_args_tostring(const char *argv[]);
std::string envp_tostring(char *envp[]);
std::vector<std::string>  envp_copy_expand(char *environ[], const char *path_ext[]);

} // end namespace fawkes

#endif
