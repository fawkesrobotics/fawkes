
/***************************************************************************
 *  qa_hostinfo.h - QA for HostInfo
 *
 *  Generated: Fri Jan 16 16:31:20 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

// Do not include in api reference
///@cond QA

#include <utils/system/hostinfo.h>
#include <cstdio>

using namespace fawkes;

int
main(int argc, char **argv)
{
  HostInfo h;

  printf("Hostname:       %s\n"
	 "Short hostname: %s\n"
	 "Domain name:    %s\n"
	 "Architecture:   %s\n"
	 "Sys Name:       %s\n"
	 "Sys Release:    %s\n"
	 "Sys Version:    %s\n",
	 h.name(), h.short_name(), h.domain(), h.arch(),
	 h.sys_name(), h.sys_release(), h.sys_version());

  return 0;
}

/// @endcond
