
/***************************************************************************
 *  interface_info.h - BlackBoard Interface Info
 *
 *  Created: Mon Mar 03 15:43:45 2008 (after topic switch)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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
#include <string>

namespace fawkes {

class Time;

class InterfaceInfo
{
 public:
  InterfaceInfo(const char *type, const char *id, const unsigned char *hash,
		unsigned int serial, bool has_writer, unsigned int num_readers,
		const std::list<std::string> &readers, const std::string &writer,
		const Time *timestamp);
  InterfaceInfo(const InterfaceInfo &i);
  ~InterfaceInfo();

  const char *                    type() const;
  const char *                    id() const;
  const unsigned char *           hash() const;
  std::string                     hash_printable() const;
  bool                            has_writer() const;
  unsigned int                    num_readers() const;
  const std::list<std::string> &  readers() const;
  const std::string &             writer() const;
  unsigned int                    serial() const;
  const Time *                    timestamp() const;

  bool operator<(const InterfaceInfo &ii) const;

 private:
  char          *__type;
  char          *__id;
  unsigned char *__hash;
  bool           __has_writer;
  unsigned int   __num_readers;
  unsigned int   __serial;
  Time          *__timestamp;
  std::list<std::string> __readers;
  std::string    __writer;
};


class InterfaceInfoList : public std::list<InterfaceInfo>
{
 public:
  void append(const char *type, const char *id, const unsigned char *hash,
	      unsigned int serial, bool has_writer, unsigned int num_readers,
	      const std::list<std::string> &readers, const std::string &writer,
	      const Time &timestamp);
};

} // end namespace fawkes

#endif
