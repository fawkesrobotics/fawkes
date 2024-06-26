
/***************************************************************************
 *  feature_blackboard.h - CLIPS blackboard feature
 *
 *  Created: Thu Oct 03 11:46:20 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef _PLUGINS_CLIPS_FEATURE_BLACKBOARD_H_
#define _PLUGINS_CLIPS_FEATURE_BLACKBOARD_H_

#include <clipsmm/value.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <list>
#include <map>
#include <string>

namespace CLIPS {
class Environment;
}

namespace fawkes {
class BlackBoard;
class Logger;
class Interface;
class Message;
class InterfaceFieldIterator;
} // namespace fawkes

class BlackboardCLIPSFeature : public fawkes::CLIPSFeature
{
public:
	BlackboardCLIPSFeature(fawkes::Logger     *logger,
	                       fawkes::BlackBoard *blackboard,
	                       bool                retract_early);
	virtual ~BlackboardCLIPSFeature();

	// for CLIPSFeature
	virtual void clips_context_init(const std::string                   &env_name,
	                                fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

private: // members
	fawkes::Logger     *logger_;
	fawkes::BlackBoard *blackboard_;
	bool                cfg_retract_early_;

	typedef std::map<std::string, std::list<fawkes::Interface *>> InterfaceMap;
	typedef struct
	{
		InterfaceMap reading;
		InterfaceMap writing;
	} Interfaces;
	std::map<std::string, Interfaces>                          interfaces_;
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> envs_;
	//which created message belongs to which interface
	std::map<fawkes::Message *, fawkes::Interface *> interface_of_msg_;

private: // methods
	void clips_blackboard_open_interface(const std::string &env_name,
	                                     const std::string &type,
	                                     const std::string &id,
	                                     bool               writing);
	void clips_blackboard_open_interface_reading(const std::string &env_name,
	                                             const std::string &type,
	                                             const std::string &id);
	void clips_blackboard_open_interface_writing(const std::string &env_name,
	                                             const std::string &type,
	                                             const std::string &id);
	void clips_blackboard_close_interface(const std::string &env_name,
	                                      const std::string &type,
	                                      const std::string &id);
	void clips_blackboard_read(const std::string &env_name);
	void clips_blackboard_write(const std::string &env_name, const std::string &uid);

	void          clips_blackboard_enable_time_read(const std::string &env_name);
	void          clips_blackboard_get_info(const std::string &env_name);
	bool          clips_assert_interface_type(const std::string &env_name,
	                                          const std::string &log_name,
	                                          fawkes::Interface *iface,
	                                          const std::string &type);
	void          clips_blackboard_preload(const std::string &env_name, const std::string &type);
	void          clips_blackboard_set(const std::string &env_name,
	                                   const std::string &uid,
	                                   const std::string &field,
	                                   CLIPS::Value       value);
	void          clips_blackboard_set_multifield(const std::string &env_name,
	                                              const std::string &uid,
	                                              const std::string &field,
	                                              CLIPS::Values      values);
	CLIPS::Value  clips_blackboard_create_msg(const std::string &env_name,
	                                          const std::string &uid,
	                                          const std::string &msg_type);
	CLIPS::Values clips_blackboard_list_msg_fields(const std::string &env_name, void *msgptr);

	void         clips_blackboard_set_msg_field(const std::string &env_name,
	                                            void              *msgptr,
	                                            const std::string &field_name,
	                                            CLIPS::Value       value);
	void         clips_blackboard_set_msg_multifield(const std::string &env_name,
	                                                 void              *msgptr,
	                                                 const std::string &field_name,
	                                                 CLIPS::Values      values);
	CLIPS::Value clips_blackboard_send_msg(const std::string &env_name, void *msgptr);

	//helper
	bool set_field(fawkes::InterfaceFieldIterator fit_begin,
	               fawkes::InterfaceFieldIterator fit_end,
	               const std::string             &env_name,
	               const std::string             &field,
	               CLIPS::Value                   value,
	               int                            index = 0);
	bool set_multifield(fawkes::InterfaceFieldIterator fit_begin,
	                    fawkes::InterfaceFieldIterator fit_end,
	                    const std::string             &env_name,
	                    const std::string             &field,
	                    CLIPS::Values                  values);
};

#endif
