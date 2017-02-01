
/***************************************************************************
 *  version.h - Fawkes version information
 *
 *  Created: Fri Aug 07 23:29:09 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_VERSION_H_
#define __CORE_VERSION_H_

#define FAWKES_VERSION_MAJOR  1
#define FAWKES_VERSION_MINOR  0
#define FAWKES_VERSION_MICRO  1

#define FAWKES_VERSION_GT(major, minor) ((FAWKES_MAJOR_VERSION  > major) || (FAWKES_MAJOR_VERSION == major) && (FAWKES_MINOR_VERSION  > minor))
#define FAWKES_VERSION_GE(major, minor) ((FAWKES_MAJOR_VERSION  > major) || (FAWKES_MAJOR_VERSION == major) && (FAWKES_MINOR_VERSION >= minor))
#define FAWKES_VERSION_EQ(major, minor) ((FAWKES_MAJOR_VERSION == major) && (FAWKES_MINOR_VERSION == minor))
#define FAWKES_VERSION_NE(major, minor) ((FAWKES_MAJOR_VERSION != major) || (FAWKES_MINOR_VERSION != minor))
#define FAWKES_VERSION_LE(major, minor) ((FAWKES_MAJOR_VERSION  < major) || (FAWKES_MAJOR_VERSION == major) && (FAWKES_MINOR_VERSION <= minor))
#define FAWKES_VERSION_LT(major, minor) ((FAWKES_MAJOR_VERSION  < major) || (FAWKES_MAJOR_VERSION == major) && (FAWKES_MINOR_VERSION  < minor))

#define FAWKES_VERSION_GT_MICRO(major, minor, micro)			\
  ((FAWKES_MAJOR_VERSION  > major) ||					\
   (FAWKES_MAJOR_VERSION == major) && (FAWKES_MINOR_VERSION  > minor) || \
   (FAWKES_MAJOR_VERSION == major) && (FAWKES_MINOR_VERSION == minor) && (FAWKES_MICRO_VERSION > minor))

#define FAWKES_VERSION_xstr(s) FAWKES_VERSION_str(s)
#define FAWKES_VERSION_str(s) #s

#define FAWKES_VERSION_STRING \
  FAWKES_VERSION_xstr(FAWKES_VERSION_MAJOR) "."	 \
  FAWKES_VERSION_xstr(FAWKES_VERSION_MINOR) "."  \
  FAWKES_VERSION_xstr(FAWKES_VERSION_MICRO)

#endif
