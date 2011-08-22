
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
   { v="naojoints", type="NaoJointPositionInterface", id="Nao Joint Positions"},
   { v="naosensors", type="NaoSensorInterface", id="Nao Sensors"}
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
local MAX_SONAR_DIST_WEAK = 0.3
local MAX_SONAR_DIST_STRONG = 0.1
local RUMBLE = {BUMPER = {LENGTH = 1000,
                          DELAY = 0,
                          STRONG = 0,
                          WEAK = 0XFFFF
                },
                AURA   = {LENGTH = 0,
                          DELAY = 0,
                          DIR = joystick.DIRECTION_UP,
                          STRONG = 0XCCCC,
                          WEAK = 0XCCCC
                }
}

local AXIS = {OMNI = {X=5, Y=4},
              WALK = {X=1, Y=0},
              HEAD = {PITCH=3, YAW=2} }

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
   return joystick:axis(AXIS.OMNI.X) ~= 0 or
          joystick:axis(AXIS.OMNI.Y) ~= 0
end

local function move_free()
   return joystick:axis(AXIS.WALK.X) ~= 0 or
          joystick:axis(AXIS.WALK.Y) ~= 0
end

local function move_head()
   return joystick:axis(AXIS.HEAD.PITCH) ~= 0 or
          joystick:axis(AXIS.HEAD.YAW) ~= 0
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
   --{"READY", "RUMBLE_TEST", true}, --use for testing rumble with analog-sticks
   {"RUMBLE_TEST", "FINAL", false},

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
   self.moved_head = false     -- true, if head was set to move
   self.moved_body = false     -- true, if nao was set to walk
   self.ultrasonic_distance = 1.0
   self.bumper_weak = 0
   self.bumper_strong = 0
end

function PLAY:loop()
   self.motion_planned = false
   self.ultrasonic_distance = naosensors:ultrasonic_distance()
   self.rumble_weak = 0
   self.rumble_strong = 0



   --process bumpers for rumbling
   if preds.left_bumper_once then
      joystick:msgq_enqueue_copy(joystick.StartRumbleMessage:new(RUMBLE.BUMPER.LENGTH,
                                                                 RUMBLE.BUMPER.DELAY,
                                                                 joystick.DIRECTION_LEFT,
                                                                 RUMBLE.BUMPER.STRONG,
                                                                 RUMBLE.BUMPER.WEAK))
   end
   if preds.right_bumper_once then
      joystick:msgq_enqueue_copy(joystick.StartRumbleMessage:new(RUMBLE.BUMPER.LENGTH,
                                                                 RUMBLE.BUMPER.DELAY,
                                                                 joystick.DIRECTION_RIGHT,
                                                                 RUMBLE.BUMPER.STRONG,
                                                                 RUMBLE.BUMPER.WEAK))
   end



   --process sonars for rumbling
   if self.ultrasonic_distance < MAX_SONAR_DIST_STRONG then
      -- multiplyer: max 1.5^2 (15cm). 1.25*0xCCCC = 0xFFFF = maximum rumble
      local multiplyer = math.pow(math.min(1.5, 10*(MAX_SONAR_DIST_STRONG - self.ultrasonic_distance)), 2)
      self.rumble_strong = math.min(0xFFFF, RUMBLE.AURA.STRONG * multiplyer)
      self.rumble_weak   = math.min(0xFFFF, RUMBLE.AURA.WEAK * 1.25)
   elseif self.ultrasonic_distance < MAX_SONAR_DIST_WEAK then
      -- multiplyer: max 1.5^2 (15cm). 1.25*0xCCCC = 0xFFFF = maximum rumble
      local multiplyer = math.pow(math.min(1.5, 10*(MAX_SONAR_DIST_WEAK - self.ultrasonic_distance)), 2)
      self.rumble_weak = math.min(0xFFFF, RUMBLE.AURA.WEAK * multiplyer)
   end
   if self.rumble_weak + self.rumble_strong > 0 then
      joystick:msgq_enqueue_copy(joystick.StartRumbleMessage:new(RUMBLE.AURA.LENGTH,
                                                                 RUMBLE.AURA.DELAY,
                                                                 RUMBLE.AURA.DIR,
                                                                 self.rumble_strong,
                                                                 self.rumble_weak))
   elseif joystick:ff_effects() ~= 0 then
      joystick:msgq_enqueue_copy(joystick.StopRumbleMessage:new())
   end



   --process movement
   if move_free() then
      self.moved_body = true
      printf("send MoveVelocity message. Move free. x:"..joystick:axis(AXIS.WALK.X).."  y:"..joystick:axis(AXIS.WALK.Y))
      self.motion_planned = true
      -- ver1: fix step-size, variable speed
      naomotion:msgq_enqueue_copy(naomotion.WalkVelocityMessage:new( FIX_STEP, 0, joystick:axis(AXIS.WALK.Y), joystick:axis(AXIS.WALK.X) ))
      -- ver2: variable step-size, fix speed
      --naomotion:msgq_enqueue_copy(naomotion.WalkVelocityMessage:new( joystick:axis(1), 0, joystick:axis(0), FIX_SPEED))

   elseif move_omni() then
      self.moved_body = true
      printf("send MoveVelocity message: move omni x:"..joystick:axis(AXIS.OMNI.X).."  y:"..joystick:axis(AXIS.OMNI.Y))
      self.motion_planned = true
      -- ver1: fix step-size, variable speed; TODO: normalize x and y, using FIX_STEP as length of direction-vector
      --naomotion:msgq_enqueue_copy(naomotion.WalkVelocityMessage:new( joystick:axis(5), joystick:axis(4), 0, 1 ))
      -- ver2: variable step-size, fix speed
      naomotion:msgq_enqueue_copy(naomotion.WalkVelocityMessage:new( joystick:axis(AXIS.OMNI.X), joystick:axis(AXIS.OMNI.Y), 0, FIX_SPEED ))
   elseif self.moved_body then
      self.moved_body = false
      printf("stop nao walking, send StopMessage")
      naomotion:msgq_enqueue_copy(naomotion.StopMessage:new())
   end



   --process head movement
   if move_head() then
      self.motion_planned = true
      self.moved_head = true

      local yaw, pitch = joystick:axis(AXIS.HEAD.YAW), joystick:axis(AXIS.HEAD.PITCH)
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
   elseif self.moved_head then
      self.moved_head = false
      printf("stop head Movement, send MoveHeadMessage(current_yaw, current_pitch)")
      naomotion:msgq_enqueue_copy(naomotion.MoveHeadMessage:new(naojoints:head_yaw(), naojoints:head_pitch(), 0.4))
   end



   -- stop condition
   if naomotion:is_moving() and
      not self.motion_planned then
      printf("send StopMessage")
      naomotion:msgq_enqueue_copy(naomotion.StopMessage:new())
   end
end

function KICK_LEFT:init()
   printf("send KickMessage(LEFT)")
   naomotion:msgq_enqueue_copy(naomotion.KickMessage:new(naomotion.LEG_LEFT, 1.0))
end

function KICK_RIGHT:init()
   printf("send KickMessage(RIGHT)")
   naomotion:msgq_enqueue_copy(naomotion.KickMessage:new(naomotion.LEG_RIGHT, 1.0))
end

function RUMBLE_TEST:init()
   self.rumble_dir = 0
   self.rumble_strong_dir = 0
   self.rumble = false
   self.strong_magnitude = 0
   self.weak_magnitude = 0
end

function RUMBLE_TEST:loop()
   self.rumble = false
   self.weak_magnitude = 0
   self.strong_magnitude = 0

   if joystick:axis(AXIS.WALK.X) > 0 then
      self.rumble = true
      self.rumble_dir = joystick.DIRECTION_UP
      self.weak_magnitude = 0X6000*math.pow(joystick:axis(AXIS.WALK.X),2)
   elseif joystick:axis(AXIS.WALK.X) < 0 then
      self.rumble = true
      self.rumble_dir = joystick.DIRECTION_DOWN
      self.weak_magnitude = 0X9000*math.pow(joystick:axis(AXIS.WALK.X),2)
   elseif joystick:axis(AXIS.WALK.Y) > 0 then
      self.rumble = true
      self.rumble_dir = joystick.DIRECTION_RIGHT
      self.weak_magnitude = 0XC000*math.pow(joystick:axis(AXIS.WALK.Y),2)
   elseif joystick:axis(AXIS.WALK.Y) < 0 then
      self.rumble = true
      self.rumble_dir = joystick.DIRECTION_LEFT
      self.weak_magnitude = 0XFFFF*math.pow(joystick:axis(AXIS.WALK.Y),2)
   end

   if joystick:axis(AXIS.HEAD.PITCH) > 0 then
      self.rumble_strong_dir = joystick.DIRECTION_UP
      self.strong_magnitude = 0X6000*math.pow(joystick:axis(AXIS.HEAD.PITCH),2)
   elseif joystick:axis(AXIS.HEAD.PITCH) < 0 then
      self.rumble_strong_dir = joystick.DIRECTION_DOWN
      self.strong_magnitude = 0X9000*math.pow(joystick:axis(AXIS.HEAD.PITCH),2)
   elseif joystick:axis(AXIS.HEAD.YAW) > 0 then
      self.rumble_strong_dir = joystick.DIRECTION_RIGHT
      self.strong_magnitude = 0XC000*math.pow(joystick:axis(AXIS.HEAD.YAW),2)
   elseif joystick:axis(AXIS.HEAD.YAW) < 0 then
      self.rumble_strong_dir = joystick.DIRECTION_LEFT
      self.strong_magnitude = 0XFFFF*math.pow(joystick:axis(AXIS.HEAD.YAW),2)
   end

   if not self.rumble and
      self.strong_magnitude > 0 then
      self.rumble = true
      self.rumble_dir = self.rumble_strong_dir
   end

   if self.rumble then
      joystick:msgq_enqueue_copy(joystick.StartRumbleMessage:new(0, 0,
                                                            self.rumble_dir,
                                                            self.strong_magnitude,
                                                            self.weak_magnitude))
   elseif joystick:ff_effects() ~= 0 then
      joystick:msgq_enqueue_copy(joystick.StopRumbleMessage:new())
   end

end