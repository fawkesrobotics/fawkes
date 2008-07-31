----------------------------------------------------------------------------
--  trivialagent.lua - a really trivial agent
--
--  Created: Tue Jul 15 23:17:13 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
require("skills.nao.search_ball")
module("skills.nao.trivialagent", skills.nao.module_init)

-- constants
local DEBUG = true --false
local IDLE_LOOP_LENGTH = 300 -- 9 secs?

-- Create FSM states
-- Indentation intentional to mark dependencies
fawkes.fsm.new_state("INITIAL")
fawkes.fsm.new_state("PRE_READY")
fawkes.fsm.new_state("READY")
fawkes.fsm.new_state("SET")
fawkes.fsm.new_state("PLAY")
  fawkes.fsm.new_state("PLAY_GOTO_BALL")
  fawkes.fsm.new_state("PLAY_SEARCH_BALL")
fawkes.fsm.new_state("PENALIZED")

-- The local finite state machine (FSM)
local FSM
local last_act_count = 0
local idle_loop_count = 0

function say(text)
  if DEBUG and speechsynth:has_writer() then
    local sm = speechsynth.SayMessage:new(text)
    speechsynth:msgq_enqueue_copy(sm)
  end
end

function toggle_ball_rec(enabled)
  tballrec:set_enabled(enabled)
  tballrec:write()
end


function short_activation()
  if chestbutton:short_activations() > 0 and chestbutton:activation_count() ~= last_act_count
  then
    last_act_count = chestbutton:activation_count()
    return true
  else
    return false
  end
end


function long_activation()
  if chestbutton:long_activations() > 0 and chestbutton:activation_count() ~= last_act_count
  then
    last_act_count = chestbutton:activation_count()
    return true
  else
    return false
  end
end


function INITIAL:loop()
  if short_activation() then return PRE_READY end
end


function PRE_READY:init()
  local m = naohw.EnableServosMessage:new()
  naohw:msgq_enqueue_copy(m)
  self.start = os.time();
end


function PRE_READY:loop()
   if os.time() - self.start > 1 then
      return READY
   end
end

function READY:init()
  -- get stiff

  -- reset head to 0 pos
  local m = naohw.SetHeadYawPitchMessage:new(20, 0, 0) --speed, yaw, pitch
  naohw:msgq_enqueue_copy(m)

  -- go to initial position
  m = naomotion.GetUpMessage:new(3.0 --[[ sec --]] );
  naomotion:msgq_enqueue_copy(m)

  toggle_ball_rec(true)
  
  -- here we go
  say("READY")
end

function READY:loop()
   if short_activation() then return SET end
end


function SET:init()
  say("SET")
 end

function SET:loop()
   if long_activation() then return PRE_READY end
  if short_activation() then
    say("Let's go")
    return PLAY
  end
end


function PLAY:process_buttons()
   if long_activation() then return PRE_READY end
   if short_activation() then return PENALIZED end
   return nil
end

function PLAY:init()
end

function PLAY:loop()
   local s = PLAY:process_buttons()
   if s then return s end

   if nao_ball:is_visible() then
      return PLAY_GOTO_BALL
   else
      return PLAY_SEARCH_BALL
   end
end


function PLAY_GOTO_BALL:init()
   --say("GOTO BALL")
  --if nao_ball:is_visible() then
    toggle_ball_rec(false)
    local m = nao_navigator.CartesianGotoMessage(nao_ball:relative_x(),
                                                 nao_ball:relative_y(),
                                                 nao_ball:yaw()) --go for goal here!
    nao_navigator:msgq_enqueue_copy(m)
  --else
  --  return PLAY
  --end
end

function PLAY_GOTO_BALL:loop()
  local s = PLAY:process_buttons()
  if s then return s end

  if idle_loop_count > IDLE_LOOP_LENGTH then
    idle_loop_count = 0
    return PLAY
  else
    idle_loop_count = idle_loop_count + 1
  end
  -- Currently we will not kick
  --if ball_kickable then
  --  kick
  --end

  -- nao_ball:visibility_history() might be used to keep walking for the
  -- last known position of the ball for a couple of cycles
      -- if relgoto(nao_ball:relative_x(), nao_ball:relative_y()) ~= S_RUNNING then
  --return PLAY
      -- end
end

function PLAY_GOTO_BALL:exit()
  toggle_ball_rec(true)
   --skills.nao.relgoto.relgoto_reset()
end

function PLAY_SEARCH_BALL:init()
   --say("Where ees the ball?")
   skills.nao.search_ball.search_ball_reset()
end

function PLAY_SEARCH_BALL:loop()
   local s = PLAY:process_buttons()
   if s then return s end

   if skills.nao.search_ball.search_ball() ~= S_RUNNING then
      return PLAY
   end
end

--function PLAY_SEARCH_BALL:exit()
--   skills.nao.search_ball.search_ball_reset()
--end

function PENALIZED:init()
   say("PENALIZED")
end

function PENALIZED:loop()
  if short_activation() then
    say("PLAY")
    return PLAY
  end
end

function trivialagent()
   FSM:loop()
   return S_RUNNING
end

function trivialagent_reset()
   FSM = fawkes.fsm.FSM:new{name="NaoFSM", start=INITIAL, debug=true}
end

-- Note that interfaces are not available here! Only when the skill is properly
-- called! This reset function is called here to initialize the FSM initially. It
-- is only done to write the init code only once. However, this means that the
-- initial FSM state cannot use any interfaces!
trivialagent_reset();

trivialagent_skill_doc = [==[Trivial Agent.
      This skill implements a trivial agent. It takes no arguments.
]==]

register_skill{name       = "trivialagent",
	       func       = trivialagent,
	       reset_func = trivialagent_reset,
	       doc        = trivialagent_skill_doc
	      };
