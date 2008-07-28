
----------------------------------------------------------------------------
--  search_ball.lua - ball searching behavior for the Nao
--
--  Created: Thu Jul 17 01:37:44 2008 (RoboCup 2008, Suzhou)
--  Copyright  2008  Tobias Kellner, Tim Niemueller
--
--  $Id$
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

require("skills.nao")
require("fawkes.fsm")
module(..., skills.nao.module_init)

-- constants
local DEBUG = false --true

local MAX_LOOK_LOOPS = 2
local MAX_TURN_LOOPS = 1
local LEFT_YAW = 0.8
local RIGHT_YAW = -0.8
local TOP_PITCH = 0.2
local BOTTOM_PITCH = 0.79
local H_EPS = 0.05
local V_EPS = 0.3
local TURN_SPEED_FAST = 20
local TURN_SPEED_SLOW = 5
local TURN_ANGLE = 1.2
local NUM_TURN_SAMPLES = 0

-- Create FSM states
-- Indentation intentional to mark dependencies
fawkes.fsm.new_state("INITIAL")
fawkes.fsm.new_state("LOOK_BOTTOM")
fawkes.fsm.new_state("LOOK_LEFT")
fawkes.fsm.new_state("LOOK_RIGHT")
fawkes.fsm.new_state("TURN")
fawkes.fsm.new_state("FAILURE")
fawkes.fsm.new_state("FINAL")

-- The local finite state machine (FSM)
local FSM

-- skill state variables
local look_loops = 0
local turn_loops = 0
local wait_for_motion = true

function say(text)
  if DEBUG and speechsynth:has_writer() then
    local sm = speechsynth.SayMessage:new(text)
    speechsynth:msgq_enqueue_copy(sm)
  end
end

function INITIAL:init()
  local m = naohw.SetHeadYawPitchMessage:new(TURN_SPEED_FAST, 0, TOP_PITCH)
  naohw:msgq_enqueue_copy(m)
end

function INITIAL:loop()
  return LOOK_BOTTOM
end

function LOOK_BOTTOM:init()
  local m = naohw.SetHeadYawPitchMessage:new(TURN_SPEED_SLOW, 0, BOTTOM_PITCH) --speed, yaw, pitch
  naohw:msgq_enqueue_copy(m)
end

function LOOK_BOTTOM:loop()
  if nao_ball:is_visible() then
    return FINAL
  end
  local angle = naohw:head_pitch()
  if angle + V_EPS >= BOTTOM_PITCH then
    return LOOK_LEFT
  end
end

function LOOK_LEFT:init()
  local m = naohw.SetHeadYawPitchMessage:new(TURN_SPEED_SLOW, LEFT_YAW, TOP_PITCH) --speed, yaw, pitch
  naohw:msgq_enqueue_copy(m)
end

function LOOK_LEFT:loop()
  if nao_ball:is_visible() then
    return FINAL
  end
  local angle = naohw:head_yaw()
  if angle + H_EPS >= LEFT_YAW then
    return LOOK_RIGHT
  end
end

function LOOK_RIGHT:init()
  local m = naohw.SetHeadYawPitchMessage:new(TURN_SPEED_SLOW, RIGHT_YAW, TOP_PITCH) --speed, yaw, pitch
  naohw:msgq_enqueue_copy(m)
end

function LOOK_RIGHT:loop()
  if nao_ball:is_visible() then
    return FINAL
  end
  local angle = naohw:head_yaw()
  if angle - H_EPS <= RIGHT_YAW then
    look_loops = look_loops + 1
    if look_loops < MAX_LOOK_LOOPS then
      return LOOK_BOTTOM
    else
      look_loops = 0
      return TURN
    end
  end
end

function TURN:init()
  if MAX_TURN_LOOPS > 0 then
    local m = hummot.TurnMessage(TURN_ANGLE, NUM_TURN_SAMPLES)
    hummot:msgq_enqueue_copy(m)
    wait_for_motion = true
  else
    wait_for_motion = false
  end
end

function TURN:loop()
  if nao_ball:is_visible() then
    return FINAL
  end
  if not hummot:is_moving() then
  --  if not wait_for_motion then
      if turn_loops < MAX_TURN_LOOPS then
        turn_loops = turn_loops + 1
        return LOOK_LEFT
      end
      local m = naohw.SetHeadYawPitchMessage:new(TURN_SPEED_FAST, 0, 0)
      naohw:msgq_enqueue_copy(m)
      return FAILURE
  --  end
  --else
  --  wait_for_motion = false
  end
end

function search_ball()
  --say("LOOKING FOR MY BALLS")
  FSM:loop()
  if FSM.current.name == "FINAL" then
    say("Ball found!")
    return S_FINAL
  end
  if FSM.current.name == "FAILURE" then
    say("Where is the damn ball?")
    return S_FAILED
  end
  return S_RUNNING
end


function search_ball_reset()
  print_debug("search_ball_reset() called")
  look_loops = 0
  turn_loops = 0
  FSM = fawkes.fsm.FSM:new{name="NaoFSM", start=INITIAL, debug=true}
end

search_ball_skill_doc = [==[Ball searching skill.
Put documentation here.
]==]

register_skill{name       = "search_ball",
               func       = search_ball,
               reset_func = search_ball_reset,
               doc        = search_ball_skill_doc
              };
