
------------------------------------------------------------------------
--  math.lua - Lua math helpers
--
--  Created: Wed Jul 16 20:02:24 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
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

require("fawkes.modinit")

--- This module provides a generic finite state machine (FSM).
-- @author Tim Niemueller
module(..., fawkes.modinit.register_all)


--- Transform 2D cartesian coordinates to 2D polar coordinates
-- @param cart_x X component of the cartesian coordinate
-- @param cart_y Y component of the cartesian coordinate
-- @return two floats, angle and distance (in that order) of the polar coordinate
function math.cart2polar2d(cart_x, cart_y)
   return  math.atan2(cart_y, cart_x), math.sqrt(cart_x * cart_x + cart_y * cart_y);
end


--- Transform 2D polar coordinates to 2D cartesian coordinates
-- @param polar_phi angle of the polar coordinate
-- @param polar_dist distance of the polar coordinate
-- @return two floats, x and y components of the cartesian coordinate (in that order)
function math.polar2cart2d(polar_phi, polar_dist)
   return polar_dist * math.cos(polar_phi), polar_dist * math.sin(polar_phi);
end


--- These  functions round x to the nearest integer, but round halfway cases away from zero.
-- @param r number to round
-- @return rounded number
function math.round(r) 
  if r >= 0 then return math.floor(r + .5) 
  else return math.ceil(r - .5) end
end

function math.normalize_mirror_rad(angle_rad)
  if angle_rad < -math.pi or angle_rad > math.pi then
    return angle_rad - 2 * math.pi * math.round(angle_rad / (2 * math.pi))
  else
    return angle_rad
  end
end

--- Determines the distance between two angle provided as radians. 
--  @param angle_rad1 first angle in radian
--  @param angle_rad2 second angle in radian
--  @return distance between the two angles
--
function math.angle_distance(angle_rad1, angle_rad2)
  return math.abs( math.normalize_mirror_rad(angle_rad1 - angle_rad2) )
end

--- Get algebraic sign (-1 or +1)
-- @param value value to get the sign from
function math.sign(value)
   if value == 0 then
      return 0
   else
      return value/math.abs(value)
   end
end

--- Calculate the length of a vector
-- @param x x-coordinate
-- @param y y-coordinate
function math.vec_length(x, y)
   return math.sqrt(x^2 + y^2)
end

math.nan = 0/0

--- Check if given value is nan
-- @param v the value to check
-- @return true if the value is nan, false otherwise
function math.isnan( v )
   return v ~= v
end
