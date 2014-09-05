/***************************************************************************
 *  skiller_feature.h - Additional skiller feature base class
 *
 *  Created: Wed Jul 16 13:16:01 2014 (on flight to Joao Pessoa)
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

#ifndef __PLUGINS_SKILLER_SKILLER_FEATURE_H_
#define __PLUGINS_SKILLER_SKILLER_FEATURE_H_

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>

namespace fawkes {
  class LuaContext;
}

class SkillerFeature
{
 public:
  virtual ~SkillerFeature();

  virtual void init_lua_context(fawkes::LuaContext *context) = 0;
  virtual void finalize_lua_context(fawkes::LuaContext *context) = 0;

};

#endif
