/***************************************************************************
 *  skiller_feature.cpp - Additional skiller feature base class
 *
 *  Created: Wed Jul 16 13:20:22 2014 (on flight to Joao Pessoa)
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

#include "skiller_feature.h"

/** @class SkillerFeature "skiller_feature.h"
 * Skiller feature base class.
 * Skiller features are additions to the skiller Lua context that can be
 * added dynamically.
 * @author Tim Niemueller
 *
 * @fn void SkillerFeature::init_lua_context(fawkes::LuaContext *context) = 0
 * Initialize a Lua context.
 * The Lua context has been initialized with the basics. Now the feature can
 * make its additions and add stuff or register as context watcher.
 * @param context Lua context to initialize
 *
 * @fn void SkillerFeature::finalize_lua_context(fawkes::LuaContext *context) = 0
 * Finalize a Lua context.
 * The Lua context will be torn down shortly and needs to be finalized,
 * for example perform any extra operations for proper finalization or
 * unregister a context watcher.
 * After this call the context may no longer be used or dereferenced.
 * @param context Lua context to finalize
 */

/** Virtual empty destructor. */
SkillerFeature::~SkillerFeature()
{
}
