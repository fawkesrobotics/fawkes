
/***************************************************************************
 *  fountain_thread.h - Fountain main thread
 *
 *  Created: Fri Nov 16 11:22:30 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_FOUNTAIN_FOUNTAIN_THREAD_H_
#define __FIREVISION_APPS_FOUNTAIN_FOUNTAIN_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/network.h>
#include <aspect/thread_producer.h>

class FuseServer;
class NetworkService;

class FountainThread
: public Thread,
  public ConfigurableAspect,
  public LoggingAspect,
  public NetworkAspect,
  public ThreadProducerAspect
{
 public:
  FountainThread();
  ~FountainThread();

  virtual void init();
  virtual void finalize();

 private:
  FuseServer *__fuse_server;
  NetworkService *__service;
};


#endif
