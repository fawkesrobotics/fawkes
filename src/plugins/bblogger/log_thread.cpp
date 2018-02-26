
/***************************************************************************
 *  log_thread.cpp - BB Logger Thread
 *
 *  Created: Sun Nov 08 00:02:09 2009
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

#include "log_thread.h"
#include "file.h"

#include <blackboard/blackboard.h>
#include <logging/logger.h>
#include <core/exceptions/system.h>
#include <interfaces/SwitchInterface.h>

#include <memory>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <fcntl.h>
#ifdef __FreeBSD__
#  include <sys/endian.h>
#elif defined(__MACH__) && defined(__APPLE__)
#  include <sys/_endian.h>
#else
#  include <endian.h>
#endif
#include <arpa/inet.h>
#include <sys/stat.h>
#include <sys/mman.h>

using namespace fawkes;

/** @class BBLoggerThread "log_thread.h"
 * BlackBoard logger thread.
 * One instance of this thread handles logging of one specific interface.
 * The plugin will spawn as many threads as there are interfaces to log. This
 * allows for maximum concurrency of the writers and avoids a serialization
 * bottle neck.
 * The log thread can operate in buffering mode. If this mode is disabled, the
 * data is written to the file within the blackboard data changed event, and
 * thus the writing operation can slow down the overall system, but memory
 * requirements are low. This is useful if a lot of data is written or if the
 * storage device is slow. If the mode is enabled, during the event the BB data
 * will be copied into another memory segment and the thread will be woken up.
 * Once the thread is running it stores all of the BB data segments bufferd
 * up to then.
 * The interface listener listens for events for a particular interface and
 * then writes the changes to the file.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param iface_uid interface UID which to log
 * @param logdir directory to store config files, must exist
 * @param buffering enable log buffering?
 * @param flushing true to flush after each written chunk
 * @param scenario ID of the log scenario
 * @param start_time time to use as start time for the log
 */
BBLoggerThread::BBLoggerThread(const char *iface_uid,
			       const char *logdir, bool buffering, bool flushing,
			       const char *scenario, fawkes::Time *start_time)
  : Thread("BBLoggerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("BBLoggerThread(%s)", iface_uid)
{
  set_coalesce_wakeups(true);
  set_name("BBLoggerThread(%s)", iface_uid);

  __buffering   = buffering;
  __flushing    = flushing;
  __uid         = strdup(iface_uid);
  __logdir      = strdup(logdir);
  __scenario    = strdup(scenario);
  __start       = new Time(start_time);
  __filename    = NULL;
  __queue_mutex = new Mutex();
  __data_size   = 0;
  __is_master   = false;
  __enabled     = true;

  __now = NULL;

  // Parse UID
  Interface::parse_uid(__uid, __type, __id);

  char date[21];
  Time now;
  struct tm *tmp = localtime(&(now.get_timeval()->tv_sec));
  strftime(date, 21, "%F-%H-%M-%S", tmp);

  if (asprintf(&__filename, "%s/%s-%s-%s-%s.log", LOGDIR, __scenario,
	       __type.c_str(), __id.c_str(), date) == -1) {
    throw OutOfMemoryException("Cannot generate log name");
  }
}


/** Destructor. */
BBLoggerThread::~BBLoggerThread()
{
  free(__uid);
  free(__logdir);
  free(__scenario);
  free(__filename);
  delete __queue_mutex;
  delete __start;
}


void
BBLoggerThread::init()
{
  __queues[0].clear();
  __queues[1].clear();
  __act_queue = 0;

  __queue_mutex = new Mutex();
  __data_size   = 0;

  __now = NULL;
  __num_data_items = 0;
  __session_start  = 0;

  // use open because fopen does not provide O_CREAT | O_EXCL
  // open read/write because of usage of mmap
  mode_t m = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  int fd   = open(__filename, O_RDWR | O_CREAT | O_EXCL, m);
  if ( ! fd ) {
    throw CouldNotOpenFileException(__filename, errno, "Failed to open log 1");
  } else {
    __f_data = fdopen(fd, "w+");
    if ( ! __f_data ) {
      throw CouldNotOpenFileException(__filename, errno, "Failed to open log 2");
    }
  }

  try {
    __iface = blackboard->open_for_reading(__type.c_str(), __id.c_str());
    __data_size = __iface->datasize();
  } catch (Exception &e) {
    fclose(__f_data);
    throw;
  }

  try {
    write_header();
  } catch (FileWriteException &e) {
    blackboard->close(__iface);
    fclose(__f_data);
    throw;
  }

  __now = new Time(clock);

  if (__is_master) {
    try {
      __switch_if = blackboard->open_for_writing<SwitchInterface>("BBLogger");
      __switch_if->set_enabled(__enabled);
      __switch_if->write();
      bbil_add_message_interface(__switch_if);
    } catch (Exception &e) {
      fclose(__f_data);
      throw;
    }
  }

  bbil_add_data_interface(__iface);
  bbil_add_writer_interface(__iface);

  blackboard->register_listener(this);

  logger->log_info(name(), "Logging %s to %s%s", __iface->uid(), __filename,
		   __is_master ? " as master" : "");
}


void
BBLoggerThread::finalize()
{
  blackboard->unregister_listener(this);
  if (__is_master) {
    blackboard->close(__switch_if);
  }
  update_header();
  fclose(__f_data);
  for (unsigned int q = 0; q < 2; ++q) {
    while (!__queues[q].empty()) {
      void *t = __queues[q].front();
      free(t);
      __queues[q].pop();
    }
  }
  delete __now;
  __now = NULL;
}


/** Get filename.
 * @return file name, valid after object instantiated, but before init() does not
 * mean that the file has been or can actually be opened
 */
const char *
BBLoggerThread::get_filename() const
{
  return __filename;
}


/** Enable or disable logging.
 * @param enabled true to enable logging, false to disable
 */
void
BBLoggerThread::set_enabled(bool enabled)
{
  if (enabled && !__enabled) {
    logger->log_info(name(), "Logging enabled");
    __session_start = __num_data_items;
  } else if (!enabled && __enabled) {
    logger->log_info(name(), "Logging disabled (wrote %u entries), flushing",
		     (__num_data_items - __session_start));
    update_header();
    fflush(__f_data);
  }

  __enabled = enabled;
}


/** Set threadlist and master status.
 * This copies the thread list and sets this thread as master thread.
 * If you intend to use this method you must do so before the thread is
 * initialized. You may only ever declare one thread as master.
 * @param thread_list list of threads to notify on enable/disable events
 */
void
BBLoggerThread::set_threadlist(fawkes::ThreadList &thread_list)
{
  __is_master = true;
  __threads   = thread_list;
}

void
BBLoggerThread::write_header()
{
  bblog_file_header header;
  memset(&header, 0, sizeof(header));
  header.file_magic   = htonl(BBLOGGER_FILE_MAGIC);
  header.file_version = htonl(BBLOGGER_FILE_VERSION);
#if __BYTE_ORDER == __BIG_ENDIAN
  header.endianess = BBLOG_BIG_ENDIAN;
#else
  header.endianess = BBLOG_LITTLE_ENDIAN;
#endif
  header.num_data_items = __num_data_items;
  strncpy(header.scenario, (const char *)__scenario, BBLOG_SCENARIO_SIZE-1);
  strncpy(header.interface_type, __iface->type(), BBLOG_INTERFACE_TYPE_SIZE-1);
  strncpy(header.interface_id, __iface->id(), BBLOG_INTERFACE_ID_SIZE-1);
  memcpy(header.interface_hash, __iface->hash(), BBLOG_INTERFACE_HASH_SIZE);
  header.data_size = __iface->datasize();
  long start_time_sec, start_time_usec;
  __start->get_timestamp(start_time_sec, start_time_usec);
  header.start_time_sec  = start_time_sec;
  header.start_time_usec = start_time_usec;
  if (fwrite(&header, sizeof(header), 1, __f_data) != 1) {
    throw FileWriteException(__filename, "Failed to write header");
  }
  fflush(__f_data);
}

/** Updates the num_data_items field in the header. */
void
BBLoggerThread::update_header()
{
  // write updated num_data_items field
#if _POSIX_MAPPED_FILES
  void *h = mmap(NULL, sizeof(bblog_file_header), PROT_WRITE, MAP_SHARED,
		 fileno(__f_data), 0);
  if (h == MAP_FAILED) {
    logger->log_warn(name(), "Failed to mmap log (%s), "
		     "not updating number of data items",
		     strerror(errno));
  } else {
    bblog_file_header *header = (bblog_file_header *)h;
    header->num_data_items = __num_data_items;
    munmap(h, sizeof(bblog_file_header));
  }
#else
  logger->log_warn(name(), "Memory mapped files not available, "
		   "not updating number of data items on close");
#endif
}

void
BBLoggerThread::write_chunk(const void *chunk)
{
  bblog_entry_header ehead;
  __now->stamp();
  Time d = *__now - *__start;
  long rel_time_sec, rel_time_usec;
  d.get_timestamp(rel_time_sec, rel_time_usec);
  ehead.rel_time_sec  = rel_time_sec;
  ehead.rel_time_usec = rel_time_usec;
  if ( (fwrite(&ehead, sizeof(ehead), 1, __f_data) == 1) &&
       (fwrite(chunk, __data_size, 1, __f_data) == 1) ) {
    if (__flushing)  fflush(__f_data);
    __num_data_items += 1;
  } else {
    logger->log_warn(name(), "Failed to write chunk");
  }
}


void
BBLoggerThread::loop()
{
  unsigned int write_queue = __act_queue;
  __queue_mutex->lock();
  __act_queue = 1 - __act_queue;
  __queue_mutex->unlock();
  LockQueue<void *> &queue = __queues[write_queue];
  //logger->log_debug(name(), "Writing %zu entries", queue.size());
  while (! queue.empty() ) {
    void *c = queue.front();
    write_chunk(c);
    free(c);
    queue.pop();
  }
}

bool
BBLoggerThread::bb_interface_message_received(Interface *interface,
					      Message *message) throw()
{
  SwitchInterface::EnableSwitchMessage *enm;
  SwitchInterface::DisableSwitchMessage *dism;

  bool enabled = true;
  if ((enm = dynamic_cast<SwitchInterface::EnableSwitchMessage *>(message)) != NULL) {
    enabled = true;
  } else if ((dism = dynamic_cast<SwitchInterface::DisableSwitchMessage *>(message)) != NULL) {
    enabled = false;
  } else {
    logger->log_debug(name(), "Unhandled message type: %s via %s",
		      message->type(), interface->uid());
  }

  for (ThreadList::iterator i = __threads.begin(); i != __threads.end(); ++i) {
    BBLoggerThread *bblt = dynamic_cast<BBLoggerThread *>(*i);
    bblt->set_enabled(enabled);
  }

  __switch_if->set_enabled(__enabled);
  __switch_if->write();

  return false;
}


void
BBLoggerThread::bb_interface_data_changed(Interface *interface) throw()
{
  if (!__enabled)  return;

  try {
    __iface->read();

    if ( __buffering ) {
      void *c = malloc(__iface->datasize());
      memcpy(c, __iface->datachunk(), __iface->datasize());
      __queue_mutex->lock();
      __queues[__act_queue].push_locked(c);
      __queue_mutex->unlock();
      wakeup();
    } else {
      __queue_mutex->lock();
      write_chunk(__iface->datachunk());
      __queue_mutex->unlock();
    }

  } catch (Exception &e) {
    logger->log_error(name(), "Exception when data changed");
    logger->log_error(name(), e);
  }
}

void
BBLoggerThread::bb_interface_writer_added(Interface *interface,
					  unsigned int instance_serial) throw()
{
  __session_start = __num_data_items;
}

void
BBLoggerThread::bb_interface_writer_removed(Interface *interface,
					    unsigned int instance_serial) throw()
{
  logger->log_info(name(), "Writer removed (wrote %u entries), flushing",
		   (__num_data_items - __session_start));
  update_header();
  fflush(__f_data);
}
