
/***************************************************************************
 *  systemrrd_thread.h - Fawkes Proc RRD Thread
 *
 *  Created: Mon Dec 17 12:54:00 2012
 *  Copyright  2012  Bastian Klingen
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

#ifndef __PLUGINS_PROCRRD_PROCRRD_THREAD_H_
#define __PLUGINS_PROCRRD_PROCRRD_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <plugins/rrd/aspect/rrd.h>
#include <config/change_handler.h>

namespace fawkes {
  class TimeWait;
}

class ProcRRDThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::RRDAspect,
  public fawkes::ConfigurationChangeHandler
{
 public:
  ProcRRDThread();
  virtual ~ProcRRDThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void add_process(const char *path, std::string pid, std::string name);
  void remove_process(const char *path);
  std::string get_process_id(const char *process);
  void get_cpu(unsigned long int* cpus);

  virtual void config_tag_changed(const char *new_tag);
  virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);
  virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
  virtual void config_value_erased(const char *path);

 private:
  fawkes::TimeWait      *__timewait;
  int                    __samplerate;
  std::string            __netinterface;
  unsigned long int     *__lastcpu;

  fawkes::RRDGraphDefinition *__net_recv_graph;    
  fawkes::RRDGraphDefinition *__net_trans_graph;    
  fawkes::RRDDefinition      *__net_rrd;

  /// @cond INTERNALS
  typedef struct {
    std::string                 pid;
    std::string                 name;
    std::string                 rrd_name;
    unsigned long int          *last_cpu;
    fawkes::RRDDefinition      *rrd;
    fawkes::RRDGraphDefinition *cpu_graph;
    fawkes::RRDGraphDefinition *mem_graph;
    fawkes::RRDGraphDefinition *io_read_graph;
    fawkes::RRDGraphDefinition *io_write_graph;
  } ProcessInfo;
  /// @endcond

  typedef std::map<std::string, ProcessInfo> ProcessMap;
  ProcessMap __processes;
};

#endif
