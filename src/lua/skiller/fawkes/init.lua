
----------------------------------------------------------------------------
--  init.lua - Skiller run-time for Fawkes
--
--  Created: Thu Mar 13 11:24:40 2008
--  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

module("skiller.fawkes", package.seeall)

require("fawkes.logprint")
require("fawkes.interface_initializer")
fawkes.logprint.init(logger)

require("interfaces.SkillerInterface")
local SkillerInterface = fawkes.SkillerInterface

local print_debug = fawkes.logprint.print_debug
local print_info  = fawkes.logprint.print_info
local print_warn  = fawkes.logprint.print_warn
local print_error = fawkes.logprint.print_error

local skiller_if
local skdbg_if
local skdbg_layouted_if

-- skill string function generated from input
local sksf

-- State information for graph publishing
local skdbg_what = "ACTIVE"
local skdbg_graphdir = "TB"
local skdbg_graphcolored = true

local loop_has_run = false

function notify_reader_removed(instance_serial)
	 if instance_serial == skiller_if:exclusive_controller() then
			print_info("Controlling interface instance was closed, revoking exclusive control")
			skiller_if:set_exclusive_controller("")
      skiller_if:set_status_timestamp(os.time())
			skiller_if:write()
	 end
end

function process_skdbg_messages()
	 while not skdbg_if:msgq_empty() do
			local m = skdbg_if:msgq_first()
			local mtype = m:type()

			if mtype == "SetGraphMessage" then
				 skdbg_what = m:graph_fsm()
				 publish_skdbg()
			elseif mtype == "SetGraphDirectionMessage" then
				 local graph_dir = m:graph_dir()
				 if graph_dir == skdbg_if.GD_BOTTOM_TOP then
						skdbg_graphdir = "BT";
				 elseif graph_dir == skdbg_if.GD_LEFT_RIGHT then
						skdbg_graphdir = "LR"
				 elseif graph_dir == skdbg_if.GD_RIGHT_LEFT then
						skdbg_graphdir = "RL"
				 else
						skdbg_graphdir = "TB"
				 end
				 publish_skdbg()
			elseif mtype == "SetGraphColoredMessage" then
				 skdbg_graphcolored = m:is_graph_colored()
				 publish_skdbg()
			end

			skdbg_if:msgq_pop()
	 end
end

function process_skiller_messages()
	 local write_skiller_if = false
	 local skill_enqueued = false

	 while not skiller_if:msgq_empty() do
			local m = skiller_if:msgq_first()
			local mtype = m:type()
			if mtype == "AcquireControlMessage" then
				 if skiller_if:exclusive_controller() == "" then
						print_debug("%s (serial %s) is new exclusive controller", m:sender_thread_name(), m:source_id():get_string())
						skiller_if:set_exclusive_controller(m:source_id():get_string())
						write_skiller_if = true
				 elseif m:is_steal_control() then
						print_warn("%s (serial %s) steals exclusive control", m:sender_thread_name(), m:source_id():get_string())
						skiller_if:set_exclusive_controller(m:source_id():get_string())
						write_skiller_if = true
				 elseif skiller_if:exclusive_controller() == m:source_id():get_string() then
						-- ignored, already has control
				 else
						print_warn("%s tried to acquire exclusive control, but another controller exists "..
											 "already", m:sender_thread_name())
				 end
			elseif mtype == "ReleaseControlMessage" then
				 if skiller_if:exclusive_controller() == m:source_id():get_string() then
						print_debug("%s releases exclusive control", m:sender_thread_name())

						-- __continuous_reset = true
						-- __last_exclusive_controller = __skiller_if->exclusive_controller()
						skiller_if:set_exclusive_controller("")
						write_skiller_if = true
				 elseif skiller_if:exclusive_controller() ~= "" then
						print_warn("%s tried to release exclusive control, but it's not the controller",
											 m:sender_thread_name())
				 end

			elseif mtype == "ExecSkillMessage" then
				 if skiller_if:exclusive_controller() == "" or skiller_if:exclusive_controller() == m:source_id():get_string() then
						if skill_enqueued then
							 print_warn("More than one skill string enqueued, ignoring previous string (%s).",
                                                                    skiller_if:skill_string())
						end
						if skiller_if:exclusive_controller() == "" then
							 if m:sender_thread_name() == "Unknown" then
									print_debug("Remote executes '%s' without any exclusive controller",
															m:skill_string())
							 else
									print_debug("%s executes '%s' without any exclusive controller",
															m:sender_thread_name(), m:skill_string())
							 end
						else
							 print_info("%s executes '%s'", m:sender_thread_name(), m:skill_string())
						end

            if sksf then
               print_info("Aborting execution of previous skill string '%s' for new goal",
                          skiller_if:skill_string())
               skillenv.reset_all()
            end

            skiller_if:set_skill_string(m:skill_string())
            skiller_if:set_msgid(m:id())
            write_skiller_if = true

            skill_enqueued = true
            sksf, err = loadstring(m:skill_string())
            if sksf then
               skillenv.reset_all()
               local sandbox = skillenv.gensandbox()
               setfenv(sksf, sandbox)
               skiller_if:set_status(SkillerInterface.S_RUNNING)
            else
               local errstr = string.format("%s|%s", m:skill_string(), err)
               print_error("Failed to compile skill string: " .. errstr)
               skiller_if:set_status(SkillerInterface.S_FAILED)
            end
				 else
						print_warn("%s tries to exec, but other thread is controller", m:sender_thread_name())
				 end

			elseif mtype == "StopExecMessage" then
				 if skiller_if:exclusive_controller() == m:source_id():get_string() then
						print_debug("Stopping execution of '%s' on request", skiller_if:skill_string())
						sksf = nil
						skillenv.reset_all()
						skiller_if:set_skill_string("")
						skiller_if:set_msgid(SkillerInterface.S_INACTIVE)
						write_skiller_if = true
						publish_skill_status()
				 else
						if m:sender_thread_name() == "Unknown" then
							 print_debug("Remote sent stop without any exclusive controller");
						else
							 print_debug("%s sent stop without any exclusive controller",
													 m:sender_thread_name())
						end
				 end
			else
				 print_warn("Unhandled message of type %s in skiller interface", type)
			end

			skiller_if:msgq_pop()
	 end

	 if write_skiller_if then

      skiller_if:set_status_timestamp(os.time())
			skiller_if:write()
	 end
end

function publish_skill_status()
	 local old_status = skiller_if:status()

	 local old_time 	= skiller_if:status_timestamp()
	 local new_status = skillenv.get_overall_status()
	 local new_time 	= os.time()

	 skiller_if:set_status_timestamp(new_time)

	 if old_status ~= new_status then
      skiller_if:set_status(new_status)

      if new_status == SkillerInterface.S_FAILED then
				 skillenv.write_fsm_error(skillenv.get_skill_fsm(skillenv.get_active_skills()), skiller_if)
			elseif new_status == SkillerInterface.S_RUNNING or new_status == SkillerInterface.S_FINAL then
				 skiller_if:set_error("")
      end
      skiller_if:write()
	 elseif old_time < new_time then
      skiller_if:write()
	 end
end

function publish_skdbg()
	 skillenv.write_skiller_debug(skdbg_if, skdbg_layouted_if, skdbg_what, skdbg_graphdir, skdbg_graphcolored)
end

function loop()
	 if not loop_has_run then
			skdbg_if:set_graph_colored(skdbg_graphcolored)
			skdbg_if:write()
			skdbg_layouted_if:set_graph_colored(skdbg_graphcolored)
			skdbg_layouted_if:write()

			loop_has_run = true
	 end


	 process_skdbg_messages()
	 process_skiller_messages()

	 if sksf then
			skillenv.reset_status()
			local ok, errmsg = xpcall(sksf, debug.traceback)

			if not ok then
				 local errstr = string.format("%s|%s", tostring(skiller_if:skill_string()), tostring(errmsg))
				 print_error("Failed to execute skill: " .. errstr)

				 skiller_if:set_error(errstr)
				 skiller_if:set_status(SkillerInterface.S_FAILED)
         skiller_if:set_status_timestamp(os.time())
				 skiller_if:write()

				 skillenv.reset_all()
				 sksf = nil
			end
			publish_skill_status()
			publish_skdbg()

			if skillenv.get_overall_status() ~= SkillerInterface.S_RUNNING then
				 -- skill ended
				 skillenv.reset_all()
				 sksf = nil
			end
	 end
end

function finalize()
	 blackboard:close(skiller_if)
	 blackboard:close(skdbg_if)
	 blackboard:close(skdbg_layouted_if)
	 skiller_if = nil
	 skdbg_if = nil
	 skdbg_layouted_if = nil

	 if not loop_has_run then
			-- Initialization failed and we are cleaned up
			fawkes.interface_initializer.preloaded_remove_without_closing()
	 end
	 skillenv.finalize()
end


function finalize_prepare()
	 print_debug("*** Lua restart in progress ***")
	 blackboard:close(skiller_if)
	 blackboard:close(skdbg_if)
	 blackboard:close(skdbg_layouted_if)
	 skiller_if = nil
	 skdbg_if = nil
	 skdbg_layouted_if = nil
end

function finalize_cancel()
	 init()
	 fawkes.interface_initializer.finalize_cancel()
end

function init()
	 skiller_if        = blackboard:open_for_writing("SkillerInterface", "Skiller")
	 skdbg_if          = blackboard:open_for_writing("SkillerDebugInterface", "Skiller")
	 skdbg_layouted_if = blackboard:open_for_writing("SkillerDebugInterface", "SkillerLayouted")

	 -- read interface once to enable keeping the
	 -- exclusive controller on a Lua context restart
	 skiller_if:read()
end
