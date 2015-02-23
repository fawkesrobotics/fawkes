/***************************************************************************
 *  navgraph_generator_thread.h - Graph-based global path planning
 *
 *  Created: Tue Sep 18 15:56:35 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAVGRAPH_GENERATOR_NAVGRAPH_GENERATOR_THREAD_H_
#define __PLUGINS_NAVGRAPH_GENERATOR_NAVGRAPH_GENERATOR_THREAD_H_

#ifdef HAVE_VISUALIZATION
#  include "visualization_thread.h"
#endif

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>
#include <blackboard/interface_listener.h>
#include <utils/math/types.h>

#include <interfaces/NavGraphGeneratorInterface.h>

class NavGraphGeneratorThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::NavGraphAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  NavGraphGeneratorThread();
  virtual ~NavGraphGeneratorThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  virtual bool bb_interface_message_received(fawkes::Interface *interface,
                                             fawkes::Message *message) throw();

 private:
  fawkes::NavGraphGeneratorInterface *navgen_if_;

  typedef struct {
    fawkes::cart_coord_2d_t                            position;
    fawkes::NavGraphGeneratorInterface::ConnectionMode conn_mode;
    std::map<std::string, std::string>                 properties;
  } PointOfInterest;

  typedef std::map<std::string, PointOfInterest>         PoiMap;
  typedef std::map<std::string, fawkes::cart_coord_2d_t> ObstacleMap;
  PoiMap      pois_;
  ObstacleMap obstacles_;

  bool                                                 copy_default_properties_;
  std::map<std::string, std::string>                   default_properties_;

  bool                    bbox_set_;
  fawkes::cart_coord_2d_t bbox_p1_;
  fawkes::cart_coord_2d_t bbox_p2_;
};

#endif
