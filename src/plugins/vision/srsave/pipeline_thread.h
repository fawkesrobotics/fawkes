
/***************************************************************************
 *  pipeline_thread.h - SrSave Pipeline Thread
 *
 *  Created: Fri Jan 22 10:50:42 2010
 *  Copyright  2005-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_SRSAVE_PIPELINE_THREAD_H_
#define __FIREVISION_APPS_SRSAVE_PIPELINE_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

namespace firevision {
  class Camera;
}

class FvSrSavePipelineThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::VisionAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  FvSrSavePipelineThread();
  virtual ~FvSrSavePipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  firevision::Camera *__cam;

  unsigned int __frame_i;
};

#endif /* __FIREVISION_APPS_SRSAVE_PIPELINE_THREAD_H_ */
