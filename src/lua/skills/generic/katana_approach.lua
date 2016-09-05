
----------------------------------------------------------------------------
--  katana_approach.lua - Katana approaching
--
--  Created: Thu Aug 30 22:32:43 2012
--  Copyright  2012  Bahram Maleki-Fard
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
name               = "katana_approach"
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = {"katana_rel"}
depends_interfaces = {
   {v = "katanaarm", id = "Katana", type = "KatanaInterface"}
}

documentation      = [==[Katana approaching

This skill tries to approach something, i.e. move the arm forward in a straight line
on the x-y pane. On failure, it tries to change the target a little bit and try
multiple approaches this way, i.e. increasing/reducing the approach distance and
moving the target up/down the z-axis a little.

All these parameters are optional:

dist=DIST (highly recommended!)
 Distance to target.
 This should allways be set, but MAY be left out.

dir=DIR
 Direction of approach. Currently only supporting "x" and "z", as
 "y" is hardly accomplishable with this 5DOF-katana

min=MIN
 Minimum approach distance.

max=MAX
 Maximum approach distance.

orth_lower=ORTH_LOWER
 Distance to lower border of orthogonal line.
 Example1: Direction is "x" => moving on xy-pane => "orth" would be on z-axis.
           Approach on xy-pane, arm would be allowed to go down by ORTH_LOWER
 Example2: Direction is "z" => moving on zy-pane => "orth" would be on x-axis.
           Approach on zy pane, arm would be allowed to go back by ORTH_LOWER
 Should be set to 0 if you want to restrict the arm from moving "down".

orth_upper=ORTH_UPPER
 see ORTH_LOWER.
 Example1: Approach on xy-pane, arm would be allowed to go up by ORTH_UPPER
 Example2: Approach on zy-pane, arm would be allowed to go forth by ORTH_UPPER

step=STEP
 The stepsize that is used to calculate new approach target.
 A smaller value results in more targets, thus eventually longer time
 before skill fails.

theta_error=THETA_ERROR
 error in the theta-value; see katana skill
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Constants
local MIN_APPROACH = 0.03 --min approach distance
local MAX_APPROACH = 0.15 --max approach distance
local ORTH_LOWER = 0.015  --lower border for orthogonal deviation
local ORTH_UPPER = 0.015  --upper border for orthogonal deviation
local STEP = 0.005        --step-length for each new approach
local THETA_ERROR = 0.5

local DIR_Z = "z"
local DIR_X = "x"

local vars = {}

-- States
fsm:define_states{
   export_to=_M,
   closure={katanaarm=katanaarm},

   {"INIT",     JumpState},
   {"APPROACH", SkillJumpState, skills={{katana_rel}},
                final_to="FINAL", fail_to="APPROACH_FAILED"},

   {"APPROACH_FAILED", JumpState}
}

-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", precond="not katanaarm:has_writer()", desc="no writer"},

   {"INIT", "APPROACH", cond=true, desc="move"},

   {"APPROACH_FAILED", "FAILED", cond="self.finished", desc="tried all approaches"},
   {"APPROACH_FAILED", "APPROACH", cond=true, desc="tried all approaches"}
}

function INIT:init()
   katanaarm:read()

   vars.min_approach = self.fsm.vars.min         or MIN_APPROACH
   vars.max_approach = self.fsm.vars.max         or MAX_APPROACH
   vars.orth_lower   = self.fsm.vars.orth_lower  or ORTH_LOWER
   vars.orth_upper   = self.fsm.vars.orth_upper  or ORTH_UPPER
   vars.step         = self.fsm.vars.step        or STEP
   vars.theta_error  = self.fsm.vars.theta_error or THETA_ERROR
   vars.direction    = self.fsm.vars.dir
   if vars.direction ~= DIR_Z then vars.direction = DIR_X end

   vars.dist = self.fsm.vars.dist or vars.max_approach

   --create vector that lies on a x-y-plane and looks at same direction as gripper
   local vector_xy = fawkes.tf.Vector3:new(1, 0, 0) -- look straight forward
   -- now look at same direction as gripper
   vector_xy = vector_xy:rotate(fawkes.tf.Vector3:new(0,0,1), katanaarm:phi() - math.pi/2)
   vars.target = vector_xy

   --create table of different approach params
   vars.targets = {}
   table.insert(vars.targets, {dist=vars.dist, orth=0.0})
   for i=-vars.orth_lower, vars.orth_upper, vars.step do
      for j=vars.min_approach, vars.max_approach, vars.step do
         table.insert(vars.targets, {dist=j, orth=i})
      end
   end
   --sort table by shortest distance to desired target
   local dist = math.vec_length(vars.dist, 0)
   function comp(w1,w2)
      local d1 = math.vec_length(w1.dist, w1.orth)
      local d2 = math.vec_length(w2.dist, w2.orth)
      if math.abs(d1 - dist) < math.abs(d2 - dist) then
         return true
      elseif (math.abs(d1 - dist) == math.abs(d2 - dist)) and (d1 > d2) then
         return true --check the further target first
      end
   end
   table.sort(vars.targets, comp)
end

function APPROACH:init()
   local x, y, z
   local target = vars.targets[1]
    if vars.direction == DIR_Z then
      if target.orth ~= 0 then
         vars.target = fawkes.tf.resize_vector(vars.target, target.orth)
         x = vars.target:x()
         y = vars.target:y()
      else
         x = 0.0
         y = 0.0
      end
      z = target.dist
   else -- vars.direction == DIR_X
      vars.target = fawkes.tf.resize_vector(vars.target, target.dist)
      x = vars.target:x()
      y = vars.target:y()
      z = target.orth
   end
   table.remove(vars.targets,1)

   self.args[katana_rel] = {
                x = x,
                y = y,
                z = z,
                straight=true,
                theta_error = vars.theta_error,
                theta=math.pi/2} --preferably look forward
end

function APPROACH_FAILED:init()
   if #vars.targets == 0 then
      self.finished = true
   end
end
