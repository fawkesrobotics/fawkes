
/***************************************************************************
 *  multi_copy.cpp - Fawkes WorldModel Multi Interface Copy Fuser
 *
 *  Created: Tue Jan 13 11:58:33 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include "multi_copy.h"

#include <core/threading/mutex_locker.h>
#include <blackboard/blackboard.h>
#include <interface/interface.h>
#include <core/exceptions/system.h>

#include <cstdlib>
#include <cstring>
#include <cstdio>

using namespace fawkes;

/** @class WorldModelMultiCopyFuser "multi_copy.h"
 * Multi interface copy fuser.
 * This fuser simply copies the data of a number of interfaces to another set
 * (of the same size) of interfaces.
 * The source interfaces are given as pattern with shell wildcards like * and ?.
 * The destination IDs is a pattern that contains exactly one "%u". This is
 * replaced with a running number for the destination interfaces. The fuser
 * registers as an observer and opens any newly created interfaces that match
 * the given pattern and creates a write with the ID like the given format for it.
 * It accounts for the case where pattern and format are similar and avoids opening
 * it's own interfaces causing an infinite loop. Interfaces are never closed.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard BlackBoard
 * @param type interface type of both interfaces
 * @param from_id_pattern pattern for ID of the interfaces to copy from
 * @param to_id_format format for ID of the interfaces to copy to
 */
WorldModelMultiCopyFuser::WorldModelMultiCopyFuser(fawkes::BlackBoard *blackboard,
						   const char *type,
						   const char *from_id_pattern,
						   const char *to_id_format)
{
  __blackboard = blackboard;
  __from_id_pattern = from_id_pattern;
  __to_id_format = to_id_format;

  std::string::size_type loc = 0;
  loc = __to_id_format.find("%");
  if ( (loc == std::string::npos) ||
       (__to_id_format.find("%", loc+1) != std::string::npos) ||
       (__to_id_format.find("%u") == std::string::npos) ) {
    throw Exception("to_id_format ('%s') must contain exactly one occurrence of %%u", to_id_format);
  }

  std::list<Interface *> exifs;
  try {
    exifs = blackboard->open_multiple_for_reading(type, from_id_pattern);
    unsigned int u = 0;
    for (std::list<Interface *>::iterator i = exifs.begin(); i != exifs.end(); ++i) {
      char *tid;
      if (asprintf(&tid, to_id_format, ++u) != -1) {
	std::string sid = tid;
	free(tid);
	Interface *to_if = blackboard->open_for_writing(type, sid.c_str());
	__ifmap[*i] = to_if;
      } else {
	throw OutOfMemoryException("Could not create interface ID, out of memory");
      }
    }
  } catch (Exception &e) {
    for (std::list<Interface *>::iterator i = exifs.begin(); i != exifs.end(); ++i) {
      blackboard->close(*i);
    }
    for (__imi = __ifmap.begin(); __imi != __ifmap.end(); ++__imi) {
      blackboard->close(__imi->second);
    }
    throw;
  }

  bbio_add_observed_create(type, from_id_pattern);
  blackboard->register_observer(this);
}


/** Destructor. */
WorldModelMultiCopyFuser::~WorldModelMultiCopyFuser()
{
  __blackboard->unregister_observer(this);

  __ifmap.lock();
  for (__imi = __ifmap.begin(); __imi != __ifmap.end(); ++__imi) {
    __blackboard->close(__imi->first);
      __blackboard->close(__imi->second);
  }
  __ifmap.clear();
  __ifmap.unlock();
}


void
WorldModelMultiCopyFuser::bb_interface_created(const char *type, const char *id) throw()
{
  unsigned int u;
  if (sscanf(id, __to_id_format.c_str(), &u) == 1) {
    // it's our own writing instance, ignore
    return;
  }

  char *tid;
  u = __ifmap.size();
  if (asprintf(&tid, __to_id_format.c_str(), u) == -1) {
    printf("Could not create ID string, asprintf() ran out of memory");
    return;
  }
  std::string sid = tid;
  free(tid);

  Interface *from_if = NULL;
  Interface *to_if   = NULL;
  
  try {
    from_if = __blackboard->open_for_reading(type, id);
    to_if   = __blackboard->open_for_writing(type, sid.c_str());

    __ifmap.lock();
    __ifmap[from_if] = to_if;
    __ifmap.unlock();
  } catch (Exception &e) {
    __blackboard->close(from_if);
    __blackboard->close(to_if);
    e.print_trace();
  }
}


void
WorldModelMultiCopyFuser::fuse()
{
  MutexLocker lock(__ifmap.mutex());
  for (__imi = __ifmap.begin(); __imi != __ifmap.end(); ++__imi) {
    if (__imi->first->has_writer()) {
      __imi->first->read();
      __imi->second->copy_values(__imi->first);
      __imi->second->write();
    }
  }
}
