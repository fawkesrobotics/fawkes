
/***************************************************************************
 *  filter_thread.h - Thread to filter laser data
 *
 *  Created: Sun Mar 13 01:11:11 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_FILTER_FILTER_THREAD_H_
#define __PLUGINS_LASER_FILTER_FILTER_THREAD_H_

#include "filters/filter.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#ifdef HAVE_TF
#  include <aspect/tf.h>
#endif

#include <string>
#include <list>
#include <vector>

namespace fawkes {
  class Laser360Interface;
  class Laser720Interface;
  class Laser1080Interface;
}

class LaserFilterThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
#ifdef HAVE_TF
  public fawkes::TransformAspect,
#endif
  public fawkes::BlackBoardAspect
{
 public:
  LaserFilterThread(std::string &cfg_name, std::string &cfg_prefix);

  virtual void init();
  virtual void finalize();
  virtual void loop();

  void wait_done();

  void set_wait_threads(std::list<LaserFilterThread *> &threads);
  void set_wait_barrier(fawkes::Barrier *barrier);

 private:
  /// @cond INTERNALS
  typedef struct {
    std::string    id;
    unsigned int   size;
    union {
      fawkes::Laser360Interface  *as360;
      fawkes::Laser720Interface  *as720;
      fawkes::Laser1080Interface *as1080;
    } interface_typed;
    fawkes::Interface *interface;
  } LaserInterface;
  /// @endcond

  void open_interfaces(std::string prefix, std::vector<LaserInterface> &ifs,
                       std::vector<LaserDataFilter::Buffer *> &bufs, bool writing);

  LaserDataFilter *  create_filter(std::string filter_name,
                                   std::string filter_type, std::string prefix,
                                   unsigned int in_data_size,
                                   std::vector<LaserDataFilter::Buffer *> &inbufs);


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::vector<LaserInterface> __in;
  std::vector<LaserInterface> __out;

  std::vector<LaserDataFilter::Buffer *>  __in_bufs;
  std::vector<LaserDataFilter::Buffer *>  __out_bufs;

  LaserDataFilter *__filter;

  std::string      __cfg_name;
  std::string      __cfg_prefix;

  std::list<LaserFilterThread *>  __wait_threads;
  bool                            __wait_done;
  fawkes::Mutex                  *__wait_mutex;
  fawkes::WaitCondition          *__wait_cond;
  fawkes::Barrier                *__wait_barrier;
};


#endif
