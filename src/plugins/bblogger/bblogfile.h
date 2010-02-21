
/***************************************************************************
 *  bblogfile.h - BlackBoard log file access convenience class
 *
 *  Created: Sun Feb 21 11:12:47 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_BBLOGGER_BBLOGFILE_H_
#define __PLUGINS_BBLOGGER_BBLOGFILE_H_

#include "file.h"

#include <core/exceptions/software.h>
#include <utils/time/time.h>

#include <cstdio>

namespace fawkes {
  class Interface;
  class BlackBoardInstanceFactory;
}

class BBLogFile {
 public:
  BBLogFile(const char *filename, bool do_sanity_check);
  BBLogFile(const char *filename, fawkes::Interface *interface = NULL,
	    bool do_sanity_check = true);
  ~BBLogFile();

  bool            has_next();
  void            read_next();
  void            read_index(unsigned int index);
  const fawkes::Time &  entry_offset() const;
  void            print_entry(FILE *outf = stdout);

  void            rewind();

  void            set_num_entries(size_t num_entries);
  void            print_info(const char *line_prefix = "", FILE *outf = stdout);

  // Header information
  uint32_t        file_version() const;
  bool            is_big_endian() const;
  uint32_t        num_data_items() const;
  const char *    scenario() const;
  const char *    interface_type() const;
  const char *    interface_id() const;
  unsigned char * interface_hash() const;
  uint32_t        data_size();
  fawkes::Time &  start_time();

  size_t          file_size() const;
  unsigned int    remaining_entries();

  static void     repair_file(const char *filename);

  void                 set_interface(fawkes::Interface *interface);
  fawkes::Interface *  interface();

  /** Get typed interface.
   * @param iface will assigned to the interface on success
   * @return interface of the given type
   * @exception TypeMismatchException thrown if interface type or ID do not match
   */
  template <class IT>
    IT *  interface(IT*& iface = 0) const
  {
    IT *rv = dynamic_cast<IT *>(__interface);
    if (rv) {
      iface = rv;
      return rv;
    } else {
      throw fawkes::TypeMismatchException("Interface types do not match.");
    }
  }

 private: // methods
  void ctor(const char *filename, bool do_sanity_check);
  void read_file_header();
  void sanity_check();
  void repair();


 private: // members
  FILE              *__f;
  bblog_file_header *__header;

  void *__ifdata;

  char *__filename;
  char *__scenario;
  char *__interface_type;
  char *__interface_id;

  fawkes::Interface *__interface;
  fawkes::BlackBoardInstanceFactory *__instance_factory;
  fawkes::Time       __start_time;
  fawkes::Time       __entry_offset;
};


#endif
