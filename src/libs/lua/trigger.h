
/***************************************************************************
 *  trigger.h - Fawkes Lua Trigger Support
 *
 *  Created: Sat Jun 21 00:48:59 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __LUA_TRIGGER_H_
#define __LUA_TRIGGER_H_

namespace fawkes {

class LuaContext;

class LuaTriggerManager
{
 public:
  LuaTriggerManager(LuaContext *lua, const char *trigger_var);
  ~LuaTriggerManager();

  void trigger(const char *event, const char *param_format = 0, ...);

 private:
  LuaContext *__lua;;
  char *__trigger_var;
};

} // end of namespace fawkes

#endif
