
/***************************************************************************
 *  hostinfo.h - hostname utilities
 *
 *  Created: Fri Jan 12 16:12:09 2007
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

#include <utils/system/hostinfo.h>

#include <core/exceptions/software.h>

#include <sys/utsname.h>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class HostInfo utils/system/hostinfo.h
 * Host information.
 * This class provides access to basic system information like hostname,
 * domain name, architecture and system information. It's basically a
 * C++ wrapper to the uname system call.
 * @author Tim Niemueller
 */

/** Constructor. */
HostInfo::HostInfo()
{
  utsname = (struct ::utsname *)malloc(sizeof(struct ::utsname));

  if ( uname(utsname) != 0 ) {
    delete utsname;
    utsname = NULL;
    throw NullPointerException("Could not call uname");
  }

  short__name = NULL;
  domain_name = NULL;

  update();
}


/** Destructor. */
HostInfo::~HostInfo()
{
  free(utsname);
  free(short__name);
  free(domain_name);
}


/** Update information.
 * Gathers the information again.
 */
void
HostInfo::update()
{
  if ( short__name != NULL ) {
    free(short__name);
  }
  if (domain_name != NULL) {
    free(domain_name);
  }

  char *dot;
  if ( (dot = strchr(utsname->nodename, '.')) == NULL ) {
    short__name  = strdup(utsname->nodename);
    domain_name = strdup("");
  } else {
    int short_length  = dot - utsname->nodename + 1;
    int domain_length = strlen(utsname->nodename) - short_length + 1;
    short__name = (char *)malloc(short_length);
    short__name[short_length - 1] = 0;
    strncpy(short__name, utsname->nodename, short_length - 1);

    domain_name = (char *)malloc(domain_length);
    domain_name[domain_length - 1] = 0;
    strncpy(domain_name, dot + 1, domain_length - 1);
  }
}


/** Get full hostname.
 * @return hostname
 */
const char *
HostInfo::name()
{
  return utsname->nodename;
}


/** Get short hostname (up to first dot).
 * @return short hostname
 */
const char *
HostInfo::short_name()
{
  return short__name;
}


/** Get domain name (after first dot or none if no dot in name).
 * @return domain name
 */
const char *
HostInfo::domain()
{
  return domain_name;
}


/** Get architecture (like i686 or x86_64).
 * @return architecture
 */
const char *
HostInfo::arch()
{
  return utsname->machine;
}


/** Get system name (like Linux).
 * @return system name
 */
const char *
HostInfo::sys_name()
{
  return utsname->sysname;
}


/** Get system release (kernel version on Linux).
 * @return system release
 */
const char *
HostInfo::sys_release()
{
  return utsname->release;
}


/** Get system version (build date on Linux).
 * @return system version
 */
const char *
HostInfo::sys_version()
{
  return utsname->version;
}

} // end namespace fawkes
