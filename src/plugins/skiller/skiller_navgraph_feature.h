/***************************************************************************
 *  skiller_navgraph_thread.h - Optional skiller access to navgraph
 *
 *  Created: Wed Jul 16 13:01:06 2014 (on flight to Joao Pessoa)
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_SKILLER_SKILLER_NAVGRAPH_FEATURE_H_
#define __PLUGINS_SKILLER_SKILLER_NAVGRAPH_FEATURE_H_

#include "skiller_feature.h"

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <navgraph/aspect/navgraph.h>

class SkillerNavGraphFeature
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::NavGraphAspect,
  public SkillerFeature
{
 public:
  SkillerNavGraphFeature();
  virtual ~SkillerNavGraphFeature();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void init_lua_context(fawkes::LuaContext *context);
  virtual void finalize_lua_context(fawkes::LuaContext *context);

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
};

#endif
