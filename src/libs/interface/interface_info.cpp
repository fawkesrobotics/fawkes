
/***************************************************************************
 *  interface_info.h - BlackBoard Interface Info
 *
 *  Created: Mon Mar 03 15:44:46 2008
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

#include <interface/interface_info.h>
#include <interface/interface.h>
#include <utils/misc/strndup.h>
#include <utils/time/time.h>

#include <cstdlib>
#include <cstring>
#include <cstdio>

namespace fawkes {

/** @class InterfaceInfo <interface/interface_info.h>
 * Interface info.
 * This class holds information about a specific interface.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param type type of the interface
 * @param id id of the interface
 * @param hash version hash
 * @param has_writer true if there is a writer, false otherwise
 * @param num_readers number of readers
 * @param serial instance serial
 * @param readers name of readers of interface
 * @param writer name of writer of interface
 * @param timestamp interface timestamp (time of last write or data timestamp)
 */
InterfaceInfo::InterfaceInfo(const char *type, const char *id, const unsigned char *hash,
			     unsigned int serial, bool has_writer, unsigned int num_readers,
			     const std::list<std::string> &readers, const std::string &writer,
			     const Time *timestamp)
{
  type_ = strndup(type, INTERFACE_TYPE_SIZE_);
  id_   = strndup(id, INTERFACE_ID_SIZE_);
  hash_ = (unsigned char *)malloc(INTERFACE_HASH_SIZE_);
  memcpy(hash_, hash, INTERFACE_HASH_SIZE_);
  has_writer_ = has_writer;
  num_readers_ = num_readers;
  serial_ = serial;
  timestamp_ = new Time(timestamp);
  readers_ = readers;
  writer_ = writer;
}


/** Copy constructor.
 * @param i info to copy
 */
InterfaceInfo::InterfaceInfo(const InterfaceInfo &i)
{
  type_ = strndup(i.type_, INTERFACE_TYPE_SIZE_);
  id_   = strndup(i.id_, INTERFACE_ID_SIZE_);
  hash_ = (unsigned char *)malloc(INTERFACE_HASH_SIZE_);
  memcpy(hash_, i.hash_, INTERFACE_HASH_SIZE_);
  has_writer_ = i.has_writer_;
  num_readers_ = i.num_readers_;
  serial_ = i.serial_;
  timestamp_ = new Time(i.timestamp_);
  readers_ = i.readers_;
  writer_  = i.writer_;
}


/** Destructor. */
InterfaceInfo::~InterfaceInfo()
{
  free(type_);
  free(id_);
  free(hash_);
  delete timestamp_;
}


/** Assignment operator.
 * @param i info to copy from
 * @return reference to this instance
 */
InterfaceInfo&
InterfaceInfo::operator=(const InterfaceInfo &i)
{
  free(type_);
  free(id_);
  free(hash_);
  delete timestamp_;

  type_ = strndup(i.type_, INTERFACE_TYPE_SIZE_);
  id_   = strndup(i.id_, INTERFACE_ID_SIZE_);
  hash_ = (unsigned char *)malloc(INTERFACE_HASH_SIZE_);
  memcpy(hash_, i.hash_, INTERFACE_HASH_SIZE_);
  has_writer_ = i.has_writer_;
  num_readers_ = i.num_readers_;
  serial_ = i.serial_;
  timestamp_ = new Time(i.timestamp_);
  readers_ = i.readers_;
  writer_  = i.writer_;

  return *this;
}


/** Get interface type.
 * @return type string
 */
const char *
InterfaceInfo::type() const
{
  return type_;
}


/** Get interface ID.
 * @return ID string
 */
const char *
InterfaceInfo::id() const
{
  return id_;
}


/** Get interface version hash.
 * @return interface version hash
 */
const unsigned char *
InterfaceInfo::hash() const
{
  return hash_;
}

/** Get interface version hash in printable format.
 * @return interface version hash printable string
 */
std::string
InterfaceInfo::hash_printable() const
{
  char phash[INTERFACE_HASH_SIZE_ * 2 + 1];
  phash[INTERFACE_HASH_SIZE_ * 2] = 0;
  for (size_t s = 0; s < INTERFACE_HASH_SIZE_; ++s) {
    snprintf(&phash[s*2], 3, "%02X", hash_[s]);
  }
  return std::string(phash);
}


/** Check if there is a writer.
 * @return true if there is a writer, false otherwise
 */
bool
InterfaceInfo::has_writer() const
{
  return has_writer_;
}


/** Get number of readers.
 * @return number of readers
 */
unsigned int
InterfaceInfo::num_readers() const
{
  return num_readers_;
}


/** Get readers of interface.
 * @return string of names of readers of this interface
 */
const std::list<std::string> &
InterfaceInfo::readers() const
{
  return readers_;
}


/** Get name of writer on interface.
 * @return name of writer owner or empty string of no writer or unknown
 */
const std::string &
InterfaceInfo::writer() const
{
  return writer_;
}


/** Get interface instance serial.
 * @return type string
 */
unsigned int
InterfaceInfo::serial() const
{
  return serial_;
}


/** Get interface timestamp.
 * @return point to interface last update time
 */
const Time *
InterfaceInfo::timestamp() const
{
  return timestamp_;
}

/** < operator
 * This compares two interface infos with respect to the less than (<) relation
 * considering the type and id of an interface.
 * An interface info A is less than an interface info B (A < B) iff
 * (A.type < B.type) or ((A.type == B.type) && A.id < B.id).
 * @param ii interface info to compare this to
 * @return true if this instance is considered less than @p ii, false otherwise
 */
bool
InterfaceInfo::operator<(const InterfaceInfo &ii) const
{
  int td = strncmp(type_, ii.type_, INTERFACE_TYPE_SIZE_);
  if ( td < 0 ) {
    return true;
  } else if (td > 0) {
    return false;
  } else {
    return (strncmp(id_, ii.id_, INTERFACE_ID_SIZE_) < 0);
  }
}


/** @class InterfaceInfoList <interface/interface_info.h>
 * Interface information list.
 * List with InterfaceInfo instances.
 * @author Tim Niemueller
 */

/** Append an interface info.
 * @param type type of the interface
 * @param id id of the interface
 * @param hash version hash
 * @param has_writer true if there is a writer, false otherwise
 * @param num_readers number of readers
 * @param serial instance serial
 * @param readers name of readers of interface
 * @param writer name of writer of interface
 * @param timestamp interface timestamp (time of last write or data timestamp)
 */
void
InterfaceInfoList::append(const char *type, const char *id, const unsigned char *hash,
			  unsigned int serial, bool has_writer, unsigned int num_readers,
			  const std::list<std::string> &readers, const std::string &writer,
			  const Time &timestamp)
{
  push_back(InterfaceInfo(type, id, hash, serial, has_writer, num_readers, readers, writer, &timestamp));
}

} // end namespace fawkes
