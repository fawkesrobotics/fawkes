
----------------------------------------------------------------------------
--  skillenv.lua - Skiller skill environment functions
--
--  Created: Fri Mar 14 22:04:43 2008
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

require("fawkes.modinit")
module(..., fawkes.modinit.register_all)
require("fawkes.logprint")

local skills        = {}
local skill_status  = { running = {}, final = {}, failed = {} }

local skill_mt = {
                   -- Possible skill status values
                   S_FINAL     = 1,
                   S_RUNNING   = 2,
                   S_FAILED    = 3,

                   -- interfaces
                   -- all vars with magic prefix interface_ get added automagically
                 }

local interfaces_by_uid = {}
local interfaces        = {}
local skill_space       = ""

-- Print skill info.
-- @param skill_entry skill entry to print
function print_skill_info(skill_module)
   print("Skill: " .. skill_module.name)
   print("======================================================================")
   print(skill_module.documentation)
   print("======================================================================")
end

-- Get skill entry.
-- Looks for the requested skill.
-- @param skill name or (wrapped) function of the skill
-- @return skill entry if found, nil otherwise.
function get_skill_module(skill)
   if ( type(skill) == "string" ) then
      for _, s in ipairs(skills) do
	 if ( s.name == skill ) then
	    return s
	 end
      end
   elseif type(skill) == "function" then
      for _, s in ipairs(skills) do
	 if ( s.wrapped_execute == skill ) then
	    return s
	 end
      end
   end

   return nil
end

-- Info about a skill.
-- @param skill skill to print the documentation string for. If nil a list of available
-- skills is printed.
function skill_info(skill)
   if skill == nil then
      print("Available skills:")
      for _, s in ipairs(skills) do
	 print(" ", s.name)
      end
   else
      local m = get_skill_module(skill)
      if ( m ~= nil ) then
	 print_skill_info(m)
      else
	 print("The queried skill has not been registered")
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
   print       = fawkes.logprint.print_info,
   print_debug = fawkes.logprint.print_debug,
   print_info  = fawkes.logprint.print_info,
   print_warn  = fawkes.logprint.print_warn,
   print_error = fawkes.logprint.print_error,
   select      = select,
   sinfo       = skill_info,
   skill_info  = skill_info,
   tostring    = tostring,
   type        = type,
   unpack      = unpack,
   xpcall      = xpcall
}


function init(skillspace)
   skill_space = skillspace
   for k,v in pairs(_G) do
      local n = string.match(k, "^interface_([%a_]+)$")
      if n then
	 interfaces_by_uid[v:uid()] = v
	 printf("Adding interface %s", v:uid())
	 interfaces[n] = v
      end
   end
end

--- Generate a sandbox for skill execution.
-- The sandbox is used in the execution thread to create a new safe environment each
-- time a skill string is executed.
-- @return table suitable to be used with setfenv
function gensandbox()
   local rv = {}
   for k,v in pairs(skill_env_template) do
      rv[k] = v
   end
   for k,v in pairs(skill_mt) do
      rv[k] = v
   end
   for _, s in ipairs(skills) do
      rv[s.name] = s.wrapped_execute
   end
   for n, i in pairs(interfaces) do
      rv[n] = i
   end

   return rv
end


-- Call reset functions.
-- This iterates over a given array of skill names or functions and executes the reset
-- function for each skill
-- @param t an array with skill names, like a member of skill_status
function reset_skills(t)
   for _,v in ipairs(t) do
      local m = get_skill_module(v)
      if m ~= nil and m.reset ~= nil then
	 print_debug("Resetting skill " .. v)
	 m.reset()
      end
   end
end


-- Reset skill status.
-- Reset the status values, but do *not* call the reset functions of the skills.
function reset_status()
   skill_status = { running = {}, final = {}, failed = {} }
end


-- Reset all.
-- Resets alls skills and the skill status.
function reset_all()
   reset_skills(skill_status.running)
   reset_skills(skill_status.final)
   reset_skills(skill_status.failed)
   reset_status()
end

-- Get current skill status.
-- @return three return values, number of running, final and failed skills.
function get_status()
   return #skill_status.running, #skill_status.final, #skill_status.failed
end

-- Top skill execution starts.
-- Internal function used in the automatically generated function wrapper to indicate
-- that a skill has started its execution.
-- @param skill_name name of the skill that is about to start
function skill_loop_begin(skill_name)
   --print("Skill " .. skill_name .. " starts execution")
end

-- Top skill execution ends.
-- Internal function used in the automatically generated function wrapper. Called if a
-- thread has ended its execution.
-- @param skill_name name of the skill which's execution stopped
-- @param status status returned by the skill
function skill_loop_end(skill_name, status)
   if ( type(status) ~= "number" ) then
      print("Skill " .. skill_name .. " did not return a valid final result.")
      return
   end

   if status == skill_mt.S_FINAL then
      --print_debug("Skill function " .. skill_name .. " is final")
      table.insert(skill_status.final, skill_name)
   elseif status == skill_mt.S_RUNNING then
      --print_debug("Skill function " .. skill_name .. " is *running*")
      table.insert(skill_status.running, skill_name)
   elseif status == skill_mt.S_FAILED then
      --print_debug("Skill function " .. skill_name .. " has failed")
      table.insert(skill_status.failed, skill_name)
   else
      print("Skill " .. skill_name .. " returned an invalid skill status (" .. status .. ")")
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
	     skill_loop_begin(name)
	     rv = {func(...)}
	     skill_loop_end(name, rv[1])
	     return unpack(rv)
	  end
end


function use_skill(module_name)
   --printf("Loading skill from module %s", module_name)
   local m = require(module_name)

   assert(m.name and type(m.name) == "string", "Skill name not set or not a string")
   assert(m.documentation and type(m.documentation) == "string",
	  "Skill documentation missing or not a string")
   assert(m.execute and type(m.execute) == "function",
	  "Skill execute() function not defined or not a function")
   assert(m.reset and type(m.reset) == "function",
	  "Skill reset() function not defined or not a function")

   --printf("Trying to add skill %s", m.name)

   if m.depends_skills then
      assert(type(m.depends_skills) == "table", "Type of depends_skills not table")
      for i,t in ipairs(m.depends_skills) do
	 assert(type(t) == "string", "Type of element in depends_skills is not string")
	 local sm = get_skill_module(t)
	 assert(sm, "Skill " .. t .. " has not been added, dependencies for " .. m.name .. " cannot be met.")
	 m[sm.name] = sm
      end
   end

   if m.depends_interfaces then
      assert(type(m.depends_interfaces) == "table", "Type of depends_interfaces not table")
      for i,t in ipairs(m.depends_interfaces) do
	 assert(type(t) == "table", "Non-table element in interface dependencies")
	 assert(t.v, "Interface dependency does not have a variable name (v) field")
	 assert(t.type, "Interface dependency does not have a type field")
	 if t.id then
	    local uid = t.type .. "::" .. t.id
	    assert(interfaces_by_uid[uid], "No interface available with the UID " .. uid ..
		                           ", required by " .. m.name)
	    m[t.v] = interfaces_by_uid[uid]
	 else
	    assert(interfaces[t.v], "No interface with the variable name " .. t.v ..
		                    ", required by " .. m.name)
	    m[t.v] = interfaces[t.v]
	 end
      end
   end

   assert(get_skill_module(m.name) == nil, "A skill with the name " .. m.name .. " already exists")
   m.wrapped_execute = create_skill_wrapper(m.name, m.execute)

   if m.init and type(m.init) == "function" then
      m.init()
   end

   table.insert(skills, m)
   printf("Successfully added skill %s to current skill space", m.name)
end


-- Initialize a skill module.
-- @param m table of the module to initialize
function module_init(m)
   fawkes.modinit.module_init(m)

   local mt = getmetatable(m)
   assert(mt == nil or mt.__index == nil, "Metatable already has an __index function/table.")

   if mt == nil then
      mt = { __index = skill_mt }
   else
      mt.__index = skill_mt
   end

   setmetatable(m, mt)
end
