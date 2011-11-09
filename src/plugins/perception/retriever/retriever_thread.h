
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

#ifndef __FIREVISION_APPS_RETRIEVER_RETRIEVER_THREAD_H_
#define __FIREVISION_APPS_RETRIEVER_RETRIEVER_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>

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
  public fawkes::VisionAspect
{
 public:
  FvRetrieverThread(const char *camera_string, const char *id);
  virtual ~FvRetrieverThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  char *__id;
  char *__camera_string;

  firevision::Camera *cam;
  firevision::SharedMemoryImageBuffer *shm;
  firevision::SeqWriter *seq_writer;
  fawkes::TimeTracker *__tt;
  unsigned int __loop_count;
  unsigned int __ttc_capture;
  unsigned int __ttc_memcpy;
  unsigned int __ttc_dispose;
  bool __cam_has_timestamp_support;

  firevision::ColorModelLookupTable *__cm;
};


#endif
