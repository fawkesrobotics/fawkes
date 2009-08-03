
------------------------------------------------------------------------
--  intercept.lua - Interception utilities
--
--  Created: Sun Apr 19 17:02:24 2008
--  Copyright  2008  Bahram Maleki-Fard
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

--- This module provides functions usable for intercepting an object.
--- Basically just use the "steps_to_int" function to get info about
--- when and where I can intercept that object as soon as possible.

module(..., fawkes.modinit.module_init)


local LOOP_TIME_SEC = 0.03
local FIELD = { x = 18, y = 12 }

-- enables/disables debut messages for debug function
local DEBUG = false;
--[[
DEBUG = true;
--]]
--- debug messages if debug enabled here
function debug(format, ...)
   if DEBUG then
      print_debug(string.format(format, ...));
   end
end



--- Checks if position is out of range
-- @param x global x-coordinate
-- @param y global y-coordinate
function oor(x, y)
   if (math.abs(x) >= FIELD.x/2) or (math.abs(y) >= FIELD.y/2) then
     return true;
   else
     return false;
   end
end

--- predict position few steps ahead
--- TODO: Change to predict on time
-- @param coord starting coordinate on an axis
-- @param vel distance object moves on that axis in 1 step
-- @param steps number of steps to look ahead
-- @param noise movement noise
function predict_steps(coord, vel, steps, noise)
   debug("predict " .. steps .." steps from " .. coord .. " with vel " .. vel)
   return coord + steps*vel -- pretty simple right now. take noise into account later, maybe use kalman filter or other stuff
end

--- Get number of steps/loops you would need to intercept the ball, optionally also
--- the predicted position
--- TODO: Change to predict on time
-- @param rob_x robot global x-coordinate
-- @param rob_y robot global y-coordinate
-- @param ball_x ball global x-coordinate
-- @param ball_y ball global y-coordinate
-- @param vel_x distance ball moves on x-axis in 1 step
-- @param vel_y distance ball moves on y-axis in 1 step
-- @param rob_vel robot velocity in m/sec
-- @param noise noise value for ball movement. not really used yet
function steps_to_int(rob_x, rob_y, ball_x, ball_y, vel_x, vel_y, rob_vel, noise)
   local robot_velocity = rob_vel or 2
   local ball_rel_x, ball_rel_y = ball_x - rob_x, ball_y - rob_y
   local ball_movement_length = math.vec_length(vel_x, vel_y)

   if ball_movement_length == 0.0 then
      debug("Ball not moving! Robot's velocity: " .. robot_velocity)
      return math.vec_length(ball_rel_x, ball_rel_y) / (robot_velocity * LOOP_TIME_SEC)
   else
      local noise_z = noise or 0.0
      -- max steps: longest_path (field diagonal) / ball_movement_length
      local max_steps = math.floor(math.vec_length(FIELD.x, FIELD.y) / ball_movement_length)
      return steps_to_intercept(max_steps, robot_velocity, rob_x, rob_y, ball_x, ball_y, ball_rel_x, ball_rel_y, vel_x, vel_y, noise_z)
   end
end

--- get number of steps/loops you would need to intercept the ball
--- TODO: Change to predict on time
-- too many params to count them here. better call "steps_to_int" anyway.
-- @return number of steps needed to intercept + predicted coordinates
function steps_to_intercept(max_steps, robot_velocity, rob_x, rob_y, ball_x, ball_y, ball_rel_x, ball_rel_y, vel_x, vel_y, noise_z)
   local cur_steps = math.ceil(max_steps / 2)
   local temp_steps = math.ceil(cur_steps / 2)

   local go_x, go_y = ball_rel_x, ball_rel_y 

   while (temp_steps >= 1) do
      temp_steps = math.ceil(temp_steps)
      debug("max: %f, cur: %f, temp: %f (while start)", max_steps, cur_steps, temp_steps)

      go_x = predict_steps(ball_x, vel_x, cur_steps, noise_z) - rob_x
      go_y = predict_steps(ball_y, vel_y, cur_steps, noise_z) - rob_y
      debug("in " .. cur_steps .." steps go_x will be " .. go_x .. ", go_y will be " .. go_y)
      debug("in " .. cur_steps .." distance will be " .. math.vec_length(go_x, go_y))

      --if position reachable for robot in cur_steps
      if ( math.vec_length(go_x, go_y) >= math.vec_length(ball_rel_x, ball_rel_x) ) then
         debug("POSITION DISTANCE INCREASES")
         cur_steps = cur_steps - temp_steps
      elseif oor(go_x + rob_x, go_y + rob_y) then
         debug("POSITION OUT OF RANGE!! ")
         cur_steps = cur_steps - temp_steps
      elseif ( math.vec_length(go_x, go_y) / (robot_velocity * LOOP_TIME_SEC) <= cur_steps)  then
         debug(" CAN GET THE BALL !! ")
         cur_steps = cur_steps - temp_steps
      else
         debug(" CAN NOT GET THE BALL!! ")
         cur_steps = cur_steps + temp_steps
      end

      temp_steps = temp_steps / 2
      debug("max: %f, cur: %f, temp: %f (while end)", max_steps, cur_steps, temp_steps)
   end

   -- last check. if not reachable go 1 step further
   go_x = predict_steps(ball_x, vel_x, cur_steps, noise_z) - rob_x
   go_y = predict_steps(ball_y, vel_y, cur_steps, noise_z) - rob_y
   debug("robot needs exactly " .. math.vec_length(go_x, go_y) / (robot_velocity * LOOP_TIME_SEC) .. "steps to get there")
   if not ( math.vec_length(go_x, go_y) / (robot_velocity * LOOP_TIME_SEC) <= cur_steps) then
      cur_steps = cur_steps + 1
      go_x = predict_steps(ball_x, vel_x, cur_steps, noise_z) - rob_x
      go_y = predict_steps(ball_y, vel_y, cur_steps, noise_z) - rob_y
   end   
   return cur_steps, go_x, go_y
end
