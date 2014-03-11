
/***************************************************************************
 *  gossip_thread.h - Robot Group Communication Plugin
 *
 *  Created: Fri Feb 28 11:09:37 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_GOSSIP_GOSSIP_THREAD_H_
#define __PLUGINS_GOSSIP_GOSSIP_THREAD_H_

#include <plugins/gossip/aspect/gossip_inifin.h>

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/network.h>
#include <aspect/aspect_provider.h>

#include <memory>

class GossipThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::AspectProviderAspect,
  public fawkes::NetworkAspect
{
 public:
  GossipThread();
  virtual ~GossipThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::string cfg_service_name_;

  std::shared_ptr<fawkes::GossipGroupManager>  group_mgr_;
  fawkes::GossipAspectIniFin                   gossip_aspect_inifin_;

};

#endif
