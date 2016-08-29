
------------------------------------------------------------------------
--  tfutils.lua - TF utility functions
--
--  Created: Tue Jun 19 22:11:15 2012 -0500
--  Copyright  2012-2014  Victor Mataré
--             2014       Tim Niemueller
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
local tf = tf
module(..., fawkes.modinit.module_init)

--- Transform 2D pose with orientation between coordinate frames.
-- @param src_pos pose in source frame
-- @param src_frame source coordinate frame
function transform(src_pos, src_frame, target_frame)
   if not tf:can_transform(src_frame, target_frame, fawkes.Time:new(0,0)) then
      -- no transform currently available
      return nil
   end

   local from_t = fawkes.tf.Vector3:new(src_pos.x, src_pos.y, 0)
   local from_r = fawkes.tf.Quaternion:new(src_pos.ori, 0, 0)
   local from_p = fawkes.tf.Pose:new(from_r, from_t)
   local from_sp = fawkes.tf.StampedPose(from_p, fawkes.Time:new(0,0), src_frame)
   local to_sp = fawkes.tf.StampedPose:new()
   tf:transform_pose(target_frame, from_sp, to_sp)

   return { x = to_sp:getOrigin():x(),
            y = to_sp:getOrigin():y(),
            ori = fawkes.tf.get_yaw(to_sp:getRotation()),
            frame = target_frame
   }
end


function is_quaternion(o)
   return type(o) == "userdata" and (
      tolua.type(o) == "fawkes::tf::Quaternion" or tolua.type(o) == "const fawkes::tf::Quaternion")
end

function is_stamped_pose(o)
   return type(o) == "userdata" and (
      tolua.type(o) == "fawkes::tf:StampedPose" or tolua.type(o) == "const fawkes::tf:StampedPose")
end

function is_pose(o)
   return type(o) == "userdata" and (
      tolua.type(o) == "fawkes::tf::Pose" or tolua.type(o) == "const fawkes::tf::Pose")
end


--- Transform 6D pose with orientation between coordinate frames.
-- @param src_pos Pose in source frame, can be a Pose or StampedPose object, or an {x,y,z,ori} table, where
--        ori may be either a Quaternion object or an {x,y,z,w} table.
-- @param src_frame Source coordinate frame
-- @return A StampedPose object if src_pos was an object, or a {x,y,z,ori} table if src_pos was a table, where
--         ori is either a Quaternion object if src.ori was an object, or an {x,y,z,w} table if src.ori was a table.
function transform6D(src_pos, src_frame, target_frame)
   if not tf:can_transform(src_frame, target_frame, fawkes.Time:new(0,0)) then
      -- no transform currently available
      return nil
   end

   local from_t
   local from_r
   local from_p
   local from_sp -- Input StampedPose

   -- Detect input data format
   if is_stamped_pose(src_pos) then
      from_sp = src_pos
   elseif is_pose(src_pos) then
      from_sp = fawkes.tf.StampedPose(src_pos, fawkes.Time:new(0,0), src_frame)
   elseif type(src_pos) == "table" then
      from_t = fawkes.tf.Vector3:new(src_pos.x, src_pos.y, src_pos.z)
      if type(src_pos.ori) == "table" then
         from_r = fawkes.tf.Quaternion:new(src_pos.ori.x, src_pos.ori.y, src_pos.ori.z, src_pos.ori.w)
      elseif is_quaternion(src_pos.ori) then
         from_r = src_pos.ori
      else
         printf("tfutils.transform6D: invalid input data: tolua.type(src_pos.ori) = %s", tolua.type(src_pos.ori))
         return nil
      end
      from_p = fawkes.tf.Pose:new(from_r, from_t)
      from_sp = fawkes.tf.StampedPose(from_p, fawkes.Time:new(0,0), src_frame)
   else
      printf("tfutils.transform6D: invalid input data: tolua.type(src_pos) = %s", tolua.type(src_pos.ori))
      return nil
   end
   
   to_sp = fawkes.tf.StampedPose:new()
   tf:transform_pose(target_frame, from_sp, to_sp)

   -- Set output data format according to input format
   if type(src_pos) == "table" then
      rv = { x = to_sp:getOrigin():x(),
             y = to_sp:getOrigin():y(),
             z = to_sp:getOrigin():z() }
      if type(src_pos.ori) == "table" then
         rv.ori = { x = to_sp:getRotation():x(),
                    y=to_sp:getRotation():y(),
                    z=to_sp:getRotation():z(),
                    w=to_sp:getRotation():w() }
      else
         rv.ori = to_sp:getRotation()
      end
   else
      rv = to_sp
   end

   return rv
end


--- Return the age of the latest transform
-- @param src_frame Source frame of transform
-- @param target_frame Target frame of transform
-- @return Age of the transform from @param src_frame to @param target_frame in seconds (as float)
function tf_age(src_frame, target_frame)
   local transform = fawkes.tf.StampedTransform:new()
   tf:lookup_transform(src_frame, target_frame, transform)
   return (fawkes.Time:new() - transform.stamp)
end






