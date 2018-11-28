
/***************************************************************************
 *  retriever_thread.h - FireVision Retriever Thread
 *
 *  Created: Tue Jun 26 17:37:38 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef _FIREVISION_APPS_RETRIEVER_RETRIEVER_THREAD_H_
#define _FIREVISION_APPS_RETRIEVER_RETRIEVER_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/vision.h>

#include <string>

namespace fawkes {
  class TimeTracker;
}
namespace firevision {
  class Camera;
  class SharedMemoryImageBuffer;
  class SeqWriter;
  class ColorModelLookupTable;
}

class FvRetrieverThread
: public fawkes::Thread,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::VisionAspect,
  public fawkes::ClockAspect
{
 public:
  FvRetrieverThread(const std::string& camera_string,
                    const std::string& cfg_name,
                    const std::string& cfg_prefix);
  virtual ~FvRetrieverThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::string cfg_name_;
  std::string cfg_prefix_;
  std::string camera_string_;
  fawkes::Time *cap_time_;

  firevision::Camera *cam;
  firevision::SharedMemoryImageBuffer *shm;
  firevision::SeqWriter *seq_writer;
  fawkes::TimeTracker *tt_;
  unsigned int loop_count_;
  unsigned int ttc_capture_;
  unsigned int ttc_memcpy_;
  unsigned int ttc_dispose_;
  bool cam_has_timestamp_support_;

  firevision::ColorModelLookupTable *cm_;
};


#endif
