
----------------------------------------------------------------------------
--  led.lua - Nao LED skill
--
--  Created: Thu Nov 06 16:57:23 2008
--  Copyright  2008  Tim Niemueller [http://www.niemueller.de]
--
----------------------------------------------------------------------------

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

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "naoled"
depends_skills     = nil
depends_interfaces = {
   { v = "chestled_red", id = "Nao LED ChestBoard/Red", type = "LedInterface" },
   { v = "chestled_green", id = "Nao LED ChestBoard/Green", type = "LedInterface" },
   { v = "chestled_blue", id = "Nao LED ChestBoard/Blue", type = "LedInterface" },
   { v = "lfootled_red", id = "Nao LED LFoot/Red", type = "LedInterface" },
   { v = "lfootled_green", id = "Nao LED LFoot/Green", type = "LedInterface" },
   { v = "lfootled_blue", id = "Nao LED LFoot/Blue", type = "LedInterface" },
   { v = "rfootled_red", id = "Nao LED RFoot/Red", type = "LedInterface" },
   { v = "rfootled_green", id = "Nao LED RFoot/Green", type = "LedInterface" },
   { v = "rfootled_blue", id = "Nao LED RFoot/Blue", type = "LedInterface" },
   { v = "ear_leds", id = "Nao LED Ears", type = "LedInterface" },
   { v = "face_leds", id = "Nao LED Face", type = "LedInterface" }
}

documentation      = [==[Nao LED setting skill.
This skill allows for setting the RGB values of the chest and feet LED
arrays.

Usage:
naoled{led=, angles=, red=, green=, blue=, value=, time_sec=}

led       string  "chest", "l_foot" or "r_foot", "l_eye" or "r_eye",
                  "l_ear" or "r_ear"
angles    array   array of angles in degrees, applicable for l_eye, r_eye,
                  l_ear, and r_ear leds. Note that you can only use a specific
                  set of angles.
red       number  value for the red LED
green     number  value for the green LED
blue      number  value for the blue LED
time_sec  number  time in seconds when to reac the intensity, may be
                  zero for immediate setting (default). This is ignored
		  by the simulation environment (no DCM).

Note that for "l_ear" and "r_ear" (ear LEDs) only the blue value will be used.
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- constants
EYE_ANGLES = {0, 45, 90, 135, 180, 225, 270, 315}
EAR_ANGLES = {0, 36, 72, 108, 144, 180, 216, 252, 288, 324}

-- Skill code
function execute(args)
   args.time_sec = args.time_sec or 0.0
   args.red      = args.red or 0.0
   args.green    = args.green or 0.0
   args.blue     = args.blue or 0.0

   args.red_ifs = {}
   args.green_ifs = {}
   args.blue_ifs = {}

   if args.led == "chest" then
      args.red_ifs.chest = chestled_red
      args.green_ifs.chest = chestled_green
      args.blue_ifs.chest = chestled_blue
   elseif args.led == "l_foot" then
      args.red_ifs.l_foot = lfootled_red
      args.green_ifs.l_foot = lfootled_green
      args.blue_ifs.l_foot = lfootled_blue
   elseif args.led == "r_foot" then
      args.red_ifs.r_foot = rfootled_red
      args.green_ifs.r_foot = rfootled_green
      args.blue_ifs.r_foot = rfootled_blue
   elseif args.led == "l_eye" then
      args.angles = args.angles or EYE_ANGLES
      for _, a in ipairs(args.angles) do
	 local s_red = "LedInterface::Nao LED Face/Red/Left/" .. a .. "Deg"
	 local s_green = "LedInterface::Nao LED Face/Green/Left/" .. a .. "Deg"
	 local s_blue = "LedInterface::Nao LED Face/Blue/Left/" .. a .. "Deg"
	 args.red_ifs["l_eye_" ..tostring(a)] = face_leds[s_red] or s_red
	 args.green_ifs["l_eye_" ..tostring(a)] = face_leds[s_green] or s_green
	 args.blue_ifs["l_eye_" ..tostring(a)] = face_leds[s_blue] or s_blue
      end
   elseif args.led == "r_eye" then
      args.angles = args.angles or EYE_ANGLES
      for _, a in ipairs(args.angles) do
	 local s_red = "LedInterface::Nao LED Face/Red/Right/" .. a .. "Deg"
	 local s_green = "LedInterface::Nao LED Face/Green/Right/" .. a .. "Deg"
	 local s_blue = "LedInterface::Nao LED Face/Blue/Right/" .. a .. "Deg"
	 args.red_ifs["r_eye_" ..tostring(a)] = face_leds[s_red] or s_red
	 args.green_ifs["r_eye_" ..tostring(a)] = face_leds[s_green] or s_green
	 args.blue_ifs["r_eye_" ..tostring(a)] = face_leds[s_blue] or s_blue
      end
   elseif args.led == "l_ear" then
      args.angles = args.angles or EAR_ANGLES
      for _, a in ipairs(args.angles) do
	 local s = "LedInterface::Nao LED Ears/Left/" .. a .. "Deg"
	 args.blue_ifs["l_ear_" ..tostring(a)] = ear_leds[s] or s
      end
   elseif args.led == "r_ear" then
      args.angles = args.angles or EAR_ANGLES
      for _, a in ipairs(args.angles) do
	 local s = "LedInterface::Nao LED Ears/Left/" .. a .. "Deg"
	 args.blue_ifs["r_ear_" ..tostring(a)] = ear_leds[s] or s
      end
   end

   for _,ci in ipairs{"red_ifs", "green_ifs", "blue_ifs"} do
      for k,i in pairs(args[ci]) do
	 if type(i) ~= "userdata" then
	    print_warn("naoled(): Interface %s=%s does not exist", tostring(k), tostring(i))
	    return S_FAILED
	 elseif not i:has_writer() then
	    print_warn("naoled(): Interface %s=%s has no writer", tostring(k), tostring(i))
	    return S_FAILED
	 end
      end
   end

   for _,c in ipairs{"red", "green", "blue"} do
      for k,i in pairs(args[c .. "_ifs"]) do
	 i:msgq_enqueue_copy(i.SetIntensityMessage:new(args.time_sec, args[c]));
      end
   end

   return S_FINAL
end


function reset()
end
