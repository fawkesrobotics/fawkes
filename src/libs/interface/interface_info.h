
/***************************************************************************
 *  interface_info.h - BlackBoard Interface Info
 *
 *  Created: Mon Mar 03 15:43:45 2008 (after topic switch)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __INTERFACE_INTERFACE_INFO_H_
#define __INTERFACE_INTERFACE_INFO_H_

#include <list>

class InterfaceInfo
{
 public:
  InterfaceInfo(const char *type, const char *id, const unsigned char *hash,
		unsigned int serial, bool has_writer, unsigned int num_readers);
  InterfaceInfo(const InterfaceInfo &i);
  ~InterfaceInfo();

  const char *           type() const;
  const char *           id() const;
  const unsigned char *  hash() const;
  bool                   has_writer() const;
  unsigned int           num_readers() const;
  unsigned int           serial() const;

 private:
  char          *__type;
  char          *__id;
  unsigned char *__hash;
  bool           __has_writer;
  unsigned int   __num_readers;
  unsigned int   __serial;
};


class InterfaceInfoList : public std::list<InterfaceInfo>
{
 public:
  void append(const char *type, const char *id, const unsigned char *hash,
	      unsigned int serial, bool has_writer, unsigned int num_readers);
};

#endif
