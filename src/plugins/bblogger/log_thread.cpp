
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

  buffering_   = buffering;
  flushing_    = flushing;
  uid_         = strdup(iface_uid);
  logdir_      = strdup(logdir);
  scenario_    = strdup(scenario);
  start_       = new Time(start_time);
  filename_    = NULL;
  queue_mutex_ = new Mutex();
  data_size_   = 0;
  is_master_   = false;
  enabled_     = true;

  now_ = NULL;

  // Parse UID
  Interface::parse_uid(uid_, type_, id_);

  char date[21];
  Time now;
  struct tm *tmp = localtime(&(now.get_timeval()->tv_sec));
  strftime(date, 21, "%F-%H-%M-%S", tmp);

  if (asprintf(&filename_, "%s/%s-%s-%s-%s.log", LOGDIR, scenario_,
	       type_.c_str(), id_.c_str(), date) == -1) {
    throw OutOfMemoryException("Cannot generate log name");
  }
}


/** Destructor. */
BBLoggerThread::~BBLoggerThread()
{
  free(uid_);
  free(logdir_);
  free(scenario_);
  free(filename_);
  delete queue_mutex_;
  delete start_;
}


void
BBLoggerThread::init()
{
  queues_[0].clear();
  queues_[1].clear();
  act_queue_ = 0;

  queue_mutex_ = new Mutex();
  data_size_   = 0;

  now_ = NULL;
  num_data_items_ = 0;
  session_start_  = 0;

  // use open because fopen does not provide O_CREAT | O_EXCL
  // open read/write because of usage of mmap
  mode_t m = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  int fd   = open(filename_, O_RDWR | O_CREAT | O_EXCL, m);
  if ( ! fd ) {
    throw CouldNotOpenFileException(filename_, errno, "Failed to open log 1");
  } else {
    f_data_ = fdopen(fd, "w+");
    if ( ! f_data_ ) {
      throw CouldNotOpenFileException(filename_, errno, "Failed to open log 2");
    }
  }

  try {
    iface_ = blackboard->open_for_reading(type_.c_str(), id_.c_str());
    data_size_ = iface_->datasize();
  } catch (Exception &e) {
    fclose(f_data_);
    throw;
  }

  try {
    write_header();
  } catch (FileWriteException &e) {
    blackboard->close(iface_);
    fclose(f_data_);
    throw;
  }

  now_ = new Time(clock);

  if (is_master_) {
    try {
      switch_if_ = blackboard->open_for_writing<SwitchInterface>("BBLogger");
      switch_if_->set_enabled(enabled_);
      switch_if_->write();
      bbil_add_message_interface(switch_if_);
    } catch (Exception &e) {
      fclose(f_data_);
      throw;
    }
  }

  bbil_add_data_interface(iface_);
  bbil_add_writer_interface(iface_);

  blackboard->register_listener(this);

  logger->log_info(name(), "Logging %s to %s%s", iface_->uid(), filename_,
		   is_master_ ? " as master" : "");
}


void
BBLoggerThread::finalize()
{
  blackboard->unregister_listener(this);
  if (is_master_) {
    blackboard->close(switch_if_);
  }
  update_header();
  fclose(f_data_);
  for (unsigned int q = 0; q < 2; ++q) {
    while (!queues_[q].empty()) {
      void *t = queues_[q].front();
      free(t);
      queues_[q].pop();
    }
  }
  delete now_;
  now_ = NULL;
}


/** Get filename.
 * @return file name, valid after object instantiated, but before init() does not
 * mean that the file has been or can actually be opened
 */
const char *
BBLoggerThread::get_filename() const
{
  return filename_;
}


/** Enable or disable logging.
 * @param enabled true to enable logging, false to disable
 */
void
BBLoggerThread::set_enabled(bool enabled)
{
  if (enabled && !enabled_) {
    logger->log_info(name(), "Logging enabled");
    session_start_ = num_data_items_;
  } else if (!enabled && enabled_) {
    logger->log_info(name(), "Logging disabled (wrote %u entries), flushing",
		     (num_data_items_ - session_start_));
    update_header();
    fflush(f_data_);
  }

  enabled_ = enabled;
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
  is_master_ = true;
  threads_   = thread_list;
}

void
BBLoggerThread::write_header()
{
  bblog_file_header header;
  memset(&header, 0, sizeof(header));
  header.file_magic   = htonl(BBLOGGER_FILE_MAGIC);
  header.file_version = htonl(BBLOGGER_FILE_VERSION);
#if BYTE_ORDER_ == BIG_ENDIAN_
  header.endianess = BBLOG_BIG_ENDIAN;
#else
  header.endianess = BBLOG_LITTLE_ENDIAN;
#endif
  header.num_data_items = num_data_items_;
  strncpy(header.scenario, (const char *)scenario_, BBLOG_SCENARIO_SIZE-1);
  strncpy(header.interface_type, iface_->type(), BBLOG_INTERFACE_TYPE_SIZE-1);
  strncpy(header.interface_id, iface_->id(), BBLOG_INTERFACE_ID_SIZE-1);
  memcpy(header.interface_hash, iface_->hash(), BBLOG_INTERFACE_HASH_SIZE);
  header.data_size = iface_->datasize();
  long start_time_sec, start_time_usec;
  start_->get_timestamp(start_time_sec, start_time_usec);
  header.start_time_sec  = start_time_sec;
  header.start_time_usec = start_time_usec;
  if (fwrite(&header, sizeof(header), 1, f_data_) != 1) {
    throw FileWriteException(filename_, "Failed to write header");
  }
  fflush(f_data_);
}

/** Updates the num_data_items field in the header. */
void
BBLoggerThread::update_header()
{
  // write updated num_data_items field
#if _POSIX_MAPPED_FILES
  void *h = mmap(NULL, sizeof(bblog_file_header), PROT_WRITE, MAP_SHARED,
		 fileno(f_data_), 0);
  if (h == MAP_FAILED) {
    logger->log_warn(name(), "Failed to mmap log (%s), "
		     "not updating number of data items",
		     strerror(errno));
  } else {
    bblog_file_header *header = (bblog_file_header *)h;
    header->num_data_items = num_data_items_;
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
  now_->stamp();
  Time d = *now_ - *start_;
  long rel_time_sec, rel_time_usec;
  d.get_timestamp(rel_time_sec, rel_time_usec);
  ehead.rel_time_sec  = rel_time_sec;
  ehead.rel_time_usec = rel_time_usec;
  if ( (fwrite(&ehead, sizeof(ehead), 1, f_data_) == 1) &&
       (fwrite(chunk, data_size_, 1, f_data_) == 1) ) {
    if (flushing_)  fflush(f_data_);
    num_data_items_ += 1;
  } else {
    logger->log_warn(name(), "Failed to write chunk");
  }
}


void
BBLoggerThread::loop()
{
  unsigned int write_queue = act_queue_;
  queue_mutex_->lock();
  act_queue_ = 1 - act_queue_;
  queue_mutex_->unlock();
  LockQueue<void *> &queue = queues_[write_queue];
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

  for (ThreadList::iterator i = threads_.begin(); i != threads_.end(); ++i) {
    BBLoggerThread *bblt = dynamic_cast<BBLoggerThread *>(*i);
    bblt->set_enabled(enabled);
  }

  switch_if_->set_enabled(enabled_);
  switch_if_->write();

  return false;
}


void
BBLoggerThread::bb_interface_data_changed(Interface *interface) throw()
{
  if (!enabled_)  return;

  try {
    iface_->read();

    if ( buffering_ ) {
      void *c = malloc(iface_->datasize());
      memcpy(c, iface_->datachunk(), iface_->datasize());
      queue_mutex_->lock();
      queues_[act_queue_].push_locked(c);
      queue_mutex_->unlock();
      wakeup();
    } else {
      queue_mutex_->lock();
      write_chunk(iface_->datachunk());
      queue_mutex_->unlock();
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
  session_start_ = num_data_items_;
}

void
BBLoggerThread::bb_interface_writer_removed(Interface *interface,
					    unsigned int instance_serial) throw()
{
  logger->log_info(name(), "Writer removed (wrote %u entries), flushing",
		   (num_data_items_ - session_start_));
  update_header();
  fflush(f_data_);
}
