
----------------------------------------------------------------------------
--  skillenv.lua - Skiller skill environment functions
--
--  Created: Fir Mar 14 22:04:43 2008
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
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software Foundation,
--  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.

require("general.utils");

module("general.skillenv", general.utils.register_all);

local skills        = {};
local skill_status  = { running = {}, final = {}, failed = {} };
local skill_mt      = {
                        -- Possible skill status values
                        S_FINAL     = 1,
			S_RUNNING   = 2,
			S_FAILED    = 3,

                        -- interfaces
                        navigator   = navigator,
                        wm_ball     = wm_ball,
                        wm_pose     = wm_pose,
                        gamestate   = gamestate
		      };


-- Print skill info.
-- @param skill_entry skill entry to print
function print_skill_info(skill_entry)
   print("Skill: " .. skill_entry.name);
   print("======================================================================");
   print(skill_entry.doc);
   print("======================================================================");
end

-- Get skill entry.
-- Looks for the requested skill.
-- @param skill name or (wrapped) function of the skill
-- @return skill entry if found, nil otherwise.
function get_skill_entry(skill)
   if ( type(skill) == "string" ) then
      for _, s in ipairs(skills) do
	 if ( s.name == skill ) then
	    return s;
	 end
      end
   elseif type(skill) == "function" then
      for _, s in ipairs(skills) do
	 if ( s.func == skill ) then
	    return s;
	 end
      end
   end

   return nil;
end

-- Info about a skill.
-- @param skill skill to print the documentation string for. If nil a list of available
-- skills is printed.
function skill_info(skill)
   print("Available skills:");
   if skill == nil then
      for _, s in ipairs(skills) do
	 print(" ", s.name);
      end
   else
      local se = get_skill_entry(skill);
      if ( se ~= nil ) then
	 print_skill_info(se);
      else
	 print("The queried skill has not been registered");
      end
   end
end

-- template for the sandbox that a skill string is executed in.
local skill_env_template = {
   -- Skiller related stuff
   logger   = logger,
   config   = config,
   clock    = clock,

   -- Packages
   math     = math,
   os       = { date = os.date, getenv = os.getenv, time = os.time, difftime = os.difftime },
   string   = string,
   table    = table,

   -- For debugging only
   skills      = skills,

   -- Functions
   assert      = assert,
   error       = error,
   ipairs      = ipairs,
   next        = next,
   print       = print,
   pairs       = pairs,
   print       = general.utils.print_info,
   print_debug = general.utils.print_debug,
   print_info  = general.utils.print_info,
   print_warn  = general.utils.print_warn,
   print_error = general.utils.print_error,
   select      = select,
   sinfo       = skill_info,
   skill_info  = skill_info,
   tostring    = tostring,
   type        = type,
   unpack      = unpack,
   xpcall      = xpcall
};


-- Generate a sandbox for skill execution.
-- The sandbox is used in the execution thread to create a new safe environment each
-- time a skill string is executed.
-- @return table suitable to be used with setfenv
function gensandbox()
   local rv = {}
   for k,v in pairs(skill_env_template) do
      rv[k] = v;
   end
   for _, s in ipairs(skills) do
      rv[s.name] = s.func;
   end
   for k,v in pairs(skill_mt) do
      rv[k] = v;
   end

   return rv;
end


-- Call reset functions.
-- This iterates over a given array of skill names or functions and executes the reset
-- function for each skill
-- @param t an array with skill names, like a member of skill_status
function reset_skills(t)
   for _,v in ipairs(t) do
      local se = get_skill_entry(v);
      if se ~= nil and se.reset_func ~= nil then
	 print_debug("Resetting skill " .. v);
	 se.reset_func();
      end
   end
end


-- Reset skill status.
-- Reset the status values, but do *not* call the reset functions of the skills.
function reset_status()
   skill_status = { running = {}, final = {}, failed = {} };
end


-- Reset all.
-- Resets alls skills and the skill status.
function reset_all()
   reset_skills(skill_status.running);
   reset_skills(skill_status.final);
   reset_skills(skill_status.failed);
   reset_status();
end

-- Get current skill status.
-- @return three return values, number of running, final and failed skills.
function get_status()
   return #skill_status.running, #skill_status.final, #skill_status.failed;
end

-- Top skill execution starts.
-- Internal function used in the automatically generated function wrapper to indicate
-- that a skill has started its execution.
-- @param skill_name name of the skill that is about to start
function skill_loop_begin(skill_name)
   print("Skill " .. skill_name .. " starts execution");
end

-- Top skill execution ends.
-- Internal function used in the automatically generated function wrapper. Called if a
-- thread has ended its execution.
-- @param skill_name name of the skill which's execution stopped
-- @param status status returned by the skill
function skill_loop_end(skill_name, status)
   if ( type(status) ~= "number" ) then
      print("Skill " .. skill_name .. " did not return a valid final result.");
      return;
   end

   if status == skill_mt.S_FINAL then
      print_debug("Skill function " .. skill_name .. " is final");
      table.insert(skill_status.final, skill_name);
   elseif status == skill_mt.S_RUNNING then
      print_debug("Skill function " .. skill_name .. " is *running*");
      table.insert(skill_status.running, skill_name);
   elseif status == skill_mt.S_FAILED then
      print_debug("Skill function " .. skill_name .. " has failed");
      table.insert(skill_status.failed, skill_name);
   else
      print("Skill " .. skill_name .. " returned an invalid skill status (" .. status .. ")");
   end
end

-- Create a skill wrapper.
-- Skills are wrapped for the sandbox. If the sandbox executes a skill it is called a
-- "top" skill, meaning that this is the highest level of execution. The skill is wrapped
-- to monitor the skill execution.
-- @param name name of the skill to wrap
-- @param func real skill function to execute
-- @return function that can be executed to execute a top skill
function create_skill_wrapper(name, func)
   return function(...)
	     skill_loop_begin(name);
	     rv = {func(...)};
	     skill_loop_end(name, rv[1]);
	     return unpack(rv);
	  end
end

-- Initialize a skill module.
-- @param m table of the module to initialize
function module_init(m)
   local mt = getmetatable(m);
   assert(mt == nil or mt.__index == nil, "Metatable already has an __index function/table.");

   if mt == nil then
      mt = { __index = skill_mt };
   else
      mt.__index = skill_mt;
   end

   setmetatable(m, mt);
end

-- Register a skill.
-- Each skill must be registered to be available in the sandbox. It expects a table with the
-- following fields:
-- name: string with the name of the skill
-- func: function to execute to run the skill
-- doc:  documentation string, be explanatory and detailed
-- reset_func: this function is optional and is called for resetting running top skills.
-- @param skillentry table with the mentioned fields
function register_skill(skillentry)
   assert(skillentry.name, "Skill entry does not have a name");
   assert(skillentry.func, "Skill " .. skillentry.name .. " does not have a skill function");
   assert(skillentry.doc, "Skill " .. skillentry.name .. " is missing documentation");

   local existing_se = get_skill_entry(skillentry.name);

   if existing_se == nil then
      local new_se = { name       = skillentry.name,
		       func       = create_skill_wrapper(skillentry.name, skillentry.func),
		       reset_func = skillentry.reset_func,
		       doc        = skillentry.doc
		    };

      table.insert(skills, new_se);
      skill_mt[skillentry.name] = skillentry.func;
      skill_mt[skillentry.name .. "_reset"] = new_se.reset_func;
      -- else already loaded
   end
end
