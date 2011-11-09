
/***************************************************************************
 *  fountain_thread.h - Fountain main thread
 *
 *  Created: Fri Nov 16 11:22:30 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_FOUNTAIN_FOUNTAIN_THREAD_H_
#define __FIREVISION_APPS_FOUNTAIN_FOUNTAIN_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/network.h>
#include <aspect/thread_producer.h>

namespace fawkes {
  class NetworkService;
}
namespace firevision {
  class FuseServer;
}

class FountainThread
: public fawkes::Thread,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::NetworkAspect,
  public fawkes::ThreadProducerAspect
{
 public:
  FountainThread();
  ~FountainThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  firevision::FuseServer *__fuse_server;
  fawkes::NetworkService *__service;
};


#endif
