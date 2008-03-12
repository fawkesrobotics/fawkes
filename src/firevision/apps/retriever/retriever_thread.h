
/***************************************************************************
 *  retriever_thread.h - FireVision Retriever Thread
 *
 *  Created: Tue Jun 26 17:37:38 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_APPS_RETRIEVER_RETRIEVER_THREAD_H_
#define __FIREVISION_APPS_RETRIEVER_RETRIEVER_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>

class Camera;
class SharedMemoryImageBuffer;
class SeqWriter;
class TimeTracker;

class FvRetrieverThread
: public Thread,
  public ConfigurableAspect,
  public LoggingAspect,
  public VisionAspect
{
 public:
  FvRetrieverThread();
  virtual ~FvRetrieverThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  Camera *cam;
  SharedMemoryImageBuffer *shm;
  SeqWriter *seq_writer;
  TimeTracker *__tt;
  unsigned int __loop_count;
  unsigned int __ttc_capture;
  unsigned int __ttc_memcpy;
  unsigned int __ttc_dispose;
};


#endif
