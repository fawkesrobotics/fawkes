
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

--- Transform 6D pose with orientation between coordinate frames.
-- @param src_pos pose in source frame
-- @param src_frame source coordinate frame
function transform6D(src_pos, src_frame, target_frame)
   if not tf:can_transform(src_frame, target_frame, fawkes.Time:new(0,0)) then
      -- no transform currently available
      return nil
   end

   local from_t = fawkes.tf.Vector3:new(src_pos.x, src_pos.y, src_pos.z)
   local from_r = fawkes.tf.Quaternion:new(src_pos.ori.x, src_pos.ori.y, src_pos.ori.z, src_pos.ori.w)
   local from_p = fawkes.tf.Pose:new(from_r, from_t)
   local from_sp = fawkes.tf.StampedPose(from_p, fawkes.Time:new(0,0), src_frame)
   local to_sp = fawkes.tf.StampedPose:new()
   tf:transform_pose(target_frame, from_sp, to_sp)

   return { x = to_sp:getOrigin():x(),
            y = to_sp:getOrigin():y(),
            z = to_sp:getOrigin():z(),
            ori = {x=to_sp:getRotation():x(), y=to_sp:getRotation():y(), z=to_sp:getRotation():z(), w=to_sp:getRotation():w()},
            frame = target_frame
   }
end
