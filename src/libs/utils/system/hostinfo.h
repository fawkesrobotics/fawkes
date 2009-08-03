
/***************************************************************************
 *  hostinfo.h - hostname utilities
 *
 *  Created: Fri Jan 12 16:08:11 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_SYSTEM_HOSTINFO_H_
#define __UTILS_SYSTEM_HOSTINFO_H_

struct utsname;

namespace fawkes {

class HostInfo
{
 public:
  HostInfo();
  ~HostInfo();

  const char *  name();
  const char *  short_name();
  const char *  domain();

  const char *  arch();

  const char *  sys_name();
  const char *  sys_release();
  const char *  sys_version();

  void update();

 private:
  struct ::utsname *utsname;
  char *short__name;
  char *domain_name;
};

} // end namespace fawkes

#endif
