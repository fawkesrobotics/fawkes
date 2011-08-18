
----------------------------------------------------------------------------
--  init.lua - Nao joystick agent
--
--  Created: Wed Aug 17 15:23:56 2011
--  Copyright  2011  Bahram Maleki-Fard
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
module(..., agentenv.module_init)

-- Crucial agent information
name               = "naojoystick"
fsm                = AgentHSM:new{name=name, debug=true, start="INIT"}
depends_skills     = {}
depends_interfaces = {
   { v="joystick", type="JoystickInterface", id="Joystick" },
   { v="naomotion", type="HumanoidMotionInterface", id="Nao Motion"},
   { v="naojoints", type="NaoJointPositionInterface", id="Nao Joint Positions"}
}

documentation      = [==[Agent to control Nao via joystick.]==]

-- Initialize as agent module
agentenv.agent_module(...)

--local Skill = AgentSkillExecJumpState
local preds = require("predicates.nao")

local FIX_SPEED = 0.6
local FIX_STEP = 0.6
local MAX_HEAD_YAW   = 1.5
local MAX_HEAD_PITCH = 1.5

local function button_kick_left()
   return joystick:pressed_buttons() == joystick.BUTTON_7
end

local function button_kick_right()
   return joystick:pressed_buttons() == joystick.BUTTON_8
end

local function button_valid()
   if joystick:pressed_buttons() > 0  then
      return button_kick_left() or
             button_kick_right()
   end
   return false
end

local function move_omni()
   return joystick:axis(4) ~= 0 or
          joystick:axis(5) ~= 0
end

local function move_free()
   return joystick:axis(0) ~= 0 or
          joystick:axis(1) ~= 0
end

local function move_head()
   return joystick:axis(2) ~= 0 or
          joystick:axis(3) ~= 0
end

local function move()
   return move_omni() or move_free() or move_head()
end

-- Setup FSM
fsm:add_transitions{
   closure={preds=preds, joystick=joystick, naomotion=naomotion},

   {"INIT", "READY", true},

   {"READY", "PLAY", cond="preds.short_button"},
   --{"READY", "PLAY", true}, --use for testing without Nao

   {"TO_PLAY", "PLAY", wait_sec=0.5},
   {"PLAY", "BUTTON", cond=button_valid, desc="action button pressed"},

   {"BUTTON", "KICK_LEFT", cond=button_kick_left, desc="left_kick"},
   {"BUTTON", "KICK_RIGHT", cond=button_kick_right, desc="right_kick"},
   {"BUTTON", "PLAY", cond=true, desc="ignore other buttons"},

   {"KICK_LEFT", "TO_PLAY", cond="not naomotion:is_moving()", desc="kick finished"},
   {"KICK_RIGHT", "TO_PLAY", cond="not naomotion:is_moving()", desc="kick finished"}
}

function INIT:init()
end

function PLAY:init()
   self.motion_planned = false -- true, if a motion is planned
end

function PLAY:loop()
   self.motion_planned = false

   --process sensors for rumbling. NICE TO HAVE

   --process movement
   if move_free() then
      printf("send MoveVelocity message. Move free. x:"..joystick:axis(1).."  y:"..joystick:axis(0))
      self.motion_planned = true
      -- ver1: fix step-size, variable speed
      naomotion:msgq_enqueue_copy(naomotion.WalkVelocityMessage:new( FIX_STEP, 0, joystick:axis(1), joystick:axis(0) ))
      -- ver2: variable step-size, fix speed
      --naomotion:msgq_enqueue_copy(naomotion.WalkVelocityMessage:new( joystick:axis(1), 0, joystick:axis(0), FIX_SPEED))

   elseif move_omni() then
      printf("send MoveVelocity message: move omni x:"..joystick:axis(5).."  y:"..joystick:axis(4))
      self.motion_planned = true
      -- ver1: fix step-size, variable speed; TODO: normalize x and y, using FIX_STEP as length of direction-vector
      --naomotion:msgq_enqueue_copy(naomotion.WalkVelocityMessage:new( joystick:axis(5), joystick:axis(4), 0, 1 ))
      -- ver2: variable step-size, fix speed
      naomotion:msgq_enqueue_copy(naomotion.WalkVelocityMessage:new( joystick:axis(5), joystick:axis(4), 0, FIX_SPEED ))
   end

   --process head movement
   if move_head() then
      self.motion_planned = true

      local yaw, pitch = joystick:axis(2), joystick:axis(3)
      local speed = math.sqrt(yaw*yaw + pitch*pitch)

      yaw = yaw + naojoints:head_yaw()
      pitch = pitch + naojoints:head_pitch()

      if yaw < 0 then
         yaw = math.max(-MAX_HEAD_YAW, yaw)
      else
         yaw = math.min(MAX_HEAD_YAW, yaw)
      end
      if pitch < 0 then
         pitch = math.max(-MAX_HEAD_PITCH, pitch)
      else
         pitch = math.min(MAX_HEAD_PITCH, pitch)
      end

      printf("send MoveHead message. yaw:"..yaw.."  pitch:"..pitch.."  speed:"..speed)
      naomotion:msgq_enqueue_copy(naomotion.MoveHeadMessage:new(yaw, pitch, speed))
   end

   -- stop condition
   if naomotion:is_moving() and
      not self.motion_planned then
      printf("send STOP message")
      naomotion:msgq_enqueue_copy(naomotion.StopMessage:new())
   end
end

function KICK_LEFT:init()
   printf("send KICK_LEFT message")
   naomotion:msgq_enqueue_copy(naomotion.KickMessage:new(naomotion.LEG_LEFT, 1.0))
end

function KICK_RIGHT:init()
   printf("send KICK_RIGHT message")
   naomotion:msgq_enqueue_copy(naomotion.KickMessage:new(naomotion.LEG_RIGHT, 1.0))
end