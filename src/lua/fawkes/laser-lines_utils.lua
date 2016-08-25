
------------------------------------------------------------------------
--  laser-lines_utils.lua - Lua laser-lines helper helpers
--
--  Created: Wed Apr 24 16:58:00 2015
--  Copyright  2015 Tobias Neumann
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

--- laser_lines_center returns the center of a laser-line
-- @param end_point_1 one of the end-points of the laser-lines interface, containing x and y
-- @param end_point_2 the other end-point of the laser-lines interface, containing x and y
-- @param bearing     the bearing of the laser-lines interface
--
-- @returns {x, y, ori} of the center of the laser-line
function laser_lines_center(end_point_1, end_point_2, bearing)
  local x   = end_point_1.x + ( end_point_2.x - end_point_1.x ) / 2
  local y   = end_point_1.y + ( end_point_2.y - end_point_1.y ) / 2
  local ori = math.normalize_mirror_rad( bearing )

  return {x=x, y=y, ori=ori}
end

--- point_in_front returns a point with the offset x, y, ori in front of the laser-line-center
-- @param center the center of the laser-line as {x, y, ori} (ori need to be the ori towards the line as given by laser-lines.center)
-- @param dist   the distance in front of the center
--
-- @returnes {x, y, ori} a point in front *distance* away of the center pointing towards the center
function point_in_front(center, distance)
  local ori_turned = math.normalize_mirror_rad(math.pi + center.ori)
  local x   = center.x + math.cos( ori_turned ) * distance
  local y   = center.y + math.sin( ori_turned ) * distance
  local ori = math.normalize_mirror_rad( center.ori )

  return {x=x, y=y, ori=ori}
end

function center(line)
   return laser_lines_center(
      {x = line:end_point_1(0), y = line:end_point_1(1)},
      {x = line:end_point_2(0), y = line:end_point_2(1)},
      line:bearing()
   )
end

