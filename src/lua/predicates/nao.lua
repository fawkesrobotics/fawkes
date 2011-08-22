
------------------------------------------------------------------------
--  nao.lua - Predicates for the Nao platform
--
--  Created: Sun Mar 08 10:16:06 2009 (Wesel am Niederrhein)
--  Copyright  2009-2011  Tim Niemueller [www.niemueller.de]
--
------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

require("fawkes.predlib")

local print=print
local string=string
local math=math

--- This predicate library provides predicates relevant for the Nao
-- @author Tim Niemueller
module(..., fawkes.predlib.module_init)

depends_interfaces = {
   {v="naosensors",  id="Nao Sensors",           type="NaoSensorInterface"},
   {v="wm_ball",     id="WM Ball",               type="ObjectPositionInterface"},
   {v="chestbutton", id="Nao Button Chest",      type="SwitchInterface"},
   {v="lfootbumper", id="Nao Button Foot Left",  type="SwitchInterface"},
   {v="rfootbumper", id="Nao Button Foot Right", type="SwitchInterface"},
   {v="penalty",     id="SPL Penalty",           type="SoccerPenaltyInterface"}
}

--- Robot is lying on its back.
function lying_on_back()
   return naosensors:accel_x() < -0.8
end

--- Robot is lying on its front.
function lying_on_front()
   return naosensors:accel_x() > 0.8
end

--- Robot is lying.
function lying()
   return lying_on_back or lying_on_front
end

-- Required for short_button and long_button predicates
predparams = {
   last_button_actcount    = chestbutton:activation_count(),
   last_lfootbump_actcount = lfootbumper:activation_count(),
   last_rfootbump_actcount = rfootbumper:activation_count()
}

--- Chest button has been pressed for a short time. This only returns true once
-- and stores false to avoid double activations.
function short_button()
   if chestbutton:short_activations() > 0
      and chestbutton:activation_count() ~= predparams.last_button_actcount then

      predparams.last_button_actcount = chestbutton:activation_count()
      return true, false
   else
      return false
   end
end

--- Chest button has been pressed for an extended time. This only returns true
-- once and stores false to avoid double activations.
function long_button()
   if chestbutton:long_activations() > 0
      and chestbutton:activation_count() ~= predparams.last_button_actcount then

      predparams.last_button_actcount = chestbutton:activation_count()
      return true, false
   else
      return false
   end
end


--- Left bumper has been pushed.
function left_bumper_once()
   if lfootbumper:activation_count() > predparams.last_lfootbump_actcount then

      predparams.last_lfootbump_actcount = lfootbumper:activation_count()
      return true, false
   else
      return false
   end
end

--- Right bumper has been pushed.
function right_bumper_once()
   if rfootbumper:activation_count() > predparams.last_rfootbump_actcount then

      predparams.last_rfootbump_actcount = rfootbumper:activation_count()
      return true, false
   else
      return false
   end
end
