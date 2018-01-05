
/***************************************************************************
 *  feature_blackboard.cpp -  CLIPS blackboard feature
 *
 *  Created: Thu Oct 03 11:48:58 2013
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

#include "feature_blackboard.h"
#include <core/threading/mutex_locker.h>
#include <blackboard/blackboard.h>
#include <blackboard/exceptions.h>
#include <logging/logger.h>
#include <utils/misc/string_conversions.h>
#include <utils/time/time.h>
#include <utils/misc/string_split.h>
#include <interface/interface_info.h>

#include <clipsmm.h>

using namespace fawkes;

/** @class BlackboardCLIPSFeature "feature_blackboard.h"
 * CLIPS blackboard feature.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger message logger
 * @param blackboard blackboard to use for opening interfaces
 * @param retract_early Retract blackboard facts at the end of the same
 *        execution cycle they have been asserted in. If false (default),
 *        blackboard facts are only retracted immediately before a new
 *        fact representing a particular interface is asserted.
 */
BlackboardCLIPSFeature::BlackboardCLIPSFeature(fawkes::Logger *logger,
                                               fawkes::BlackBoard *blackboard,
                                               bool retract_early)
: CLIPSFeature("blackboard"), logger_(logger), blackboard_(blackboard),
  cfg_retract_early_(retract_early)
{
}


/** Destructor. */
BlackboardCLIPSFeature::~BlackboardCLIPSFeature()
{
  for (auto &iface_map : interfaces_) {
    for (auto &iface_list : iface_map.second.reading) {
      for (auto iface : iface_list.second) {
	blackboard_->close(iface);
      }
    }
    for (auto &iface_list : iface_map.second.writing) {
      for (auto iface : iface_list.second) {
	blackboard_->close(iface);
      }
    }
  }
  interfaces_.clear();
  envs_.clear();
}


void
BlackboardCLIPSFeature::clips_context_init(const std::string &env_name,
					   fawkes::LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;
  clips->evaluate("(path-load \"blackboard.clp\")");
  clips->add_function("blackboard-enable-time-read",
    sigc::slot<void>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_enable_time_read),
        env_name)
    )
  );
  clips->add_function("blackboard-open",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_open_interface_reading),
        env_name)
    )
  );
  clips->add_function("blackboard-open-reading",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_open_interface_reading),
        env_name)
    )
  );
  clips->add_function("blackboard-open-writing",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_open_interface_writing),
        env_name)
    )
  );
  clips->add_function("blackboard-close",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_close_interface),
        env_name)
    )
  );
  clips->add_function("blackboard-preload",
    sigc::slot<void, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_preload),
        env_name)
    )
  );
  clips->add_function("blackboard-read",
    sigc::slot<void>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_read),
	env_name)
    )
  );
  clips->add_function("blackboard-write",
    sigc::slot<void, std::string>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_write),
	env_name)
    )
  );
  clips->add_function("blackboard-get-info",
    sigc::slot<void>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_get_info),
	env_name)
    )
  );
  clips->add_function("blackboard-set",
    sigc::slot<void, std::string, std::string, CLIPS::Value>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_set),
	env_name)
    )
  );
  clips->add_function("blackboard-set-multifield",
    sigc::slot<void, std::string, std::string, CLIPS::Values>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_set_multifield),
	env_name)
    )
  );
  clips->add_function("blackboard-create-msg",
    sigc::slot<CLIPS::Value, std::string, std::string>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_create_msg),
	env_name)
    )
  );
  clips->add_function("blackboard-list-msg-fields",
    sigc::slot<CLIPS::Values, void *>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_list_msg_fields),
	env_name)
    )
  );
  clips->add_function("blackboard-set-msg-field",
    sigc::slot<void, void *, std::string, CLIPS::Value>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_set_msg_field),
	env_name)
    )
  );
  clips->add_function("blackboard-set-msg-multifield",
    sigc::slot<void, void *, std::string, CLIPS::Values>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_set_msg_multifield),
	env_name)
    )
  );
  clips->add_function("blackboard-send-msg",
    sigc::slot<CLIPS::Value, void *>(
      sigc::bind<0>(
	sigc::mem_fun(*this, &BlackboardCLIPSFeature::clips_blackboard_send_msg),
	env_name)
    )
  );
}

void
BlackboardCLIPSFeature::clips_context_destroyed(const std::string &env_name)
{
  if (interfaces_.find(env_name) != interfaces_.end()) {
    for (auto &iface_map : interfaces_[env_name].reading) {
      for (auto iface : iface_map.second) {
	logger_->log_debug(("BBCLIPS|" + env_name).c_str(), "Closing reading interface %s",
			   iface->uid());
	blackboard_->close(iface);
      }
    }
    for (auto &iface_map : interfaces_[env_name].writing) {
      for (auto iface : iface_map.second) {
	logger_->log_debug(("BBCLIPS|" + env_name).c_str(), "Closing writing interface %s",
			   iface->uid());
	blackboard_->close(iface);
      }
    }
    interfaces_.erase(env_name);
  }
  envs_.erase(env_name);
}


void
BlackboardCLIPSFeature::clips_blackboard_enable_time_read(std::string env_name)
{
  if (envs_.find(env_name) == envs_.end()) {
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(),
		      "Cannot enable reading for environment %s "
		     "(not defined)", env_name.c_str());
    return;
  }

  std::string bb_read_defrule =
    "(defrule blackboard-read\n"
    "  (declare (salience 1000))\n"
    "  (time $?)\n"
    "  =>\n"
    "  (blackboard-read)\n"
    ")";

  fawkes::MutexLocker lock(envs_[env_name].objmutex_ptr());
  envs_[env_name]->build(bb_read_defrule);
}


bool
BlackboardCLIPSFeature::clips_assert_interface_type(std::string &env_name, std::string &log_name,
						    fawkes::Interface *iface, std::string &type)
{
  std::string deftemplate =
    "(deftemplate " + type + "\n" +
    "  (slot id (type STRING))\n" +
    "  (multislot time (type INTEGER) (cardinality 2 2))\n";

  InterfaceFieldIterator f, f_end = iface->fields_end();

  for (f = iface->fields(); f != f_end; ++f) {
    std::string type;

    switch (f.get_type()) {
    case IFT_BOOL:
      deftemplate += std::string() +
	"  (" + ((f.get_length() > 1) ? "multi" : "") + "slot " + f.get_name() +
	" (type SYMBOL) (allowed-values TRUE FALSE))\n";
      break;

    case IFT_INT8:
    case IFT_UINT8:
    case IFT_INT16:
    case IFT_UINT16:
    case IFT_INT32:
    case IFT_UINT32:
    case IFT_INT64:
    case IFT_UINT64:
    case IFT_BYTE:
      deftemplate += std::string() +
	"  (" + ((f.get_length() > 1) ? "multi" : "") + "slot " + f.get_name() +
	" (type INTEGER))\n";
      break;

    case IFT_FLOAT:
    case IFT_DOUBLE:
      deftemplate += std::string() +
	"  (" + ((f.get_length() > 1) ? "multi" : "") + "slot " + f.get_name() +
	" (type FLOAT))\n";
      break;

    case IFT_STRING:
      deftemplate += std::string() +
        "  (slot " + f.get_name() +
	" (type STRING))\n";
      break;

    case IFT_ENUM:
      deftemplate += std::string() +
	"  (" + ((f.get_length() > 1) ? "multi" : "") + "slot " + f.get_name() +
	" (type SYMBOL))\n";
      break;
    }
  }

  deftemplate += ")";

  std::string retract;
  std::string logstr;

  if (cfg_retract_early_) {
    retract =
      "(defrule " + type + "-cleanup\n" +
      "  (declare (salience -10000))\n" +
      "  ?f <- (" + type + ")\n" +
      "  =>\n"
      "  (retract ?f)\n"
      ")";
    logstr = "Defrule";
  }
  else {
    retract =
      "(deffunction " + type + "-cleanup-late (?id)\n"
      "  (delayed-do-for-all-facts ((?f " + type + "))\n"
      "    (eq ?f:id ?id)\n"
      "    (retract ?f)\n"
      "  )\n"
      ")";
    logstr = "Deffunction";
  }

  if (envs_[env_name]->build(deftemplate) && envs_[env_name]->build(retract)) {
    logger_->log_debug(log_name.c_str(), "Deftemplate:\n%s", deftemplate.c_str());
    logger_->log_debug(log_name.c_str(), "%s:\n%s", logstr.c_str(), retract.c_str());
    return true;
  } else {
    logger_->log_warn(log_name.c_str(), "Defining blackboard type for %s in %s failed",
		      type.c_str(), env_name.c_str());
    return false;
  }
}


void
BlackboardCLIPSFeature::clips_blackboard_preload(std::string env_name, std::string type)
{
  std::string name = "BBCLIPS|" + env_name;

  if (envs_.find(env_name) == envs_.end()) {
    logger_->log_warn(name.c_str(), "Environment %s has not been registered "
		      "for blackboard feature", env_name.c_str());
    return;
  }

  if (interfaces_[env_name].reading.find(type) == interfaces_[env_name].reading.end() &&
      interfaces_[env_name].writing.find(type) == interfaces_[env_name].writing.end())
  {
    // no interface of this type registered yet, add deftemplate for it
    Interface *iface = NULL;
    try {
      iface = blackboard_->open_for_reading(type.c_str(), "__clips_blackboard_preload__");      
      clips_assert_interface_type(env_name, name, iface, type);
      blackboard_->close(iface);
      interfaces_[env_name].reading.insert(std::make_pair(type, std::list<fawkes::Interface *>()));
    } catch (Exception &e) {
      logger_->log_warn(name.c_str(), "Failed to preload interface type %s, "
			"exception follows", type.c_str());
      logger_->log_warn(name.c_str(), e);
      return;
    }
  }
}


void
BlackboardCLIPSFeature::clips_blackboard_open_interface(std::string env_name,
							std::string type, std::string id,
							bool writing)
{
	std::string name  = "BBCLIPS|" + env_name;
	std::string owner = "CLIPS:" + env_name;

	if (envs_.find(env_name) == envs_.end()) {
		logger_->log_warn(name.c_str(), "Environment %s has not been registered "
		                  "for blackboard feature", env_name.c_str());
		return;
	}

	fawkes::LockPtr<CLIPS::Environment> clips = envs_[env_name];

	Interface *iface = NULL;
	InterfaceMap &iface_map =
		writing ? interfaces_[env_name].writing : interfaces_[env_name].reading;

	if (iface_map.find(type) == iface_map.end()) {
		// no interface of this type registered yet, add deftemplate for it
		try {
			if (writing) {
				iface = blackboard_->open_for_writing(type.c_str(), id.c_str(), owner.c_str());
			} else {
				iface = blackboard_->open_for_reading(type.c_str(), id.c_str(), owner.c_str());
			}
		} catch (Exception &e) {
			logger_->log_warn(name.c_str(), "Failed to open interface %s:%s, exception follows",
			                  type.c_str(), id.c_str());
			logger_->log_warn(name.c_str(), e);
			return;
		}

		if (! clips_assert_interface_type(env_name, name, iface, type)) {
			blackboard_->close(iface);
		} else {
			logger_->log_info(name.c_str(), "Added interface %s for %s", iface->uid(),
			                  iface->is_writer() ? "writing" : "reading");
			iface_map.insert(std::make_pair(type, std::list<fawkes::Interface *>(1, iface)));
			fawkes::MutexLocker lock(clips.objmutex_ptr());
			clips->assert_fact_f("(blackboard-interface (id \"%s\") (type \"%s\") (uid \"%s\") "
			                     "                      (hash \"%s\") (serial %u) (writing %s))",
			                     iface->id(), iface->type(), iface->uid(), iface->hash_printable(),
			                     iface->serial(), writing ? "TRUE" : "FALSE");
		}
	} else {
		auto &iface_list = iface_map[type];
		if (std::none_of(iface_list.begin(), iface_list.end(),
		                 [&type, &id](const Interface *i)->bool {
			                 return (type == i->type()) && (id == i->id());
		                 }))
		{
			try {
				if (writing) {
					iface = blackboard_->open_for_writing(type.c_str(), id.c_str(), owner.c_str());
				} else {
					iface = blackboard_->open_for_reading(type.c_str(), id.c_str(), owner.c_str());
				}
				iface_map[type].push_back(iface);
				logger_->log_info(name.c_str(), "Added interface %s for %s", iface->uid(),
				                  iface->is_writer() ? "writing" : "reading");
				fawkes::MutexLocker lock(clips.objmutex_ptr());
				clips->assert_fact_f("(blackboard-interface (id \"%s\") (type \"%s\") (uid \"%s\") "
				                     "                      (hash \"%s\") (serial %u) (writing %s))",
				                     iface->id(), iface->type(), iface->uid(), iface->hash_printable(),
				                     iface->serial(), writing ? "TRUE" : "FALSE");
			} catch (Exception &e) {
				logger_->log_warn(name.c_str(), "Failed to open interface %s:%s, exception follows",
				                  type.c_str(), id.c_str());
				logger_->log_warn(name.c_str(), e);
				return;
			}
		}
	}
}


void
BlackboardCLIPSFeature::clips_blackboard_open_interface_reading(std::string env_name,
								std::string type, std::string id)
{
  clips_blackboard_open_interface(env_name, type, id, /* writing */ false);
}

void
BlackboardCLIPSFeature::clips_blackboard_open_interface_writing(std::string env_name,
								std::string type, std::string id)
{
  clips_blackboard_open_interface(env_name, type, id, /* writing */ true);
}


void
BlackboardCLIPSFeature::clips_blackboard_close_interface(std::string env_name,
							 std::string type, std::string id)
{
  std::string name = "BBCLIPS|" + env_name;

  if (envs_.find(env_name) == envs_.end()) {
    logger_->log_warn(name.c_str(), "Environment %s has not been registered "
		     "for blackboard feature", env_name.c_str());
    return;
  }

  if (interfaces_[env_name].reading.find(type) != interfaces_[env_name].reading.end()) {
    auto &l = interfaces_[env_name].reading[type];
    auto iface_it = find_if(l.begin(), l.end(),
			    [&id] (const Interface *iface) { return id == iface->id(); });
    if (iface_it != l.end()) {
      blackboard_->close(*iface_it);
      l.erase(iface_it);
      // do NOT remove the list, even if empty, because we need to remember
      // that we already built the deftemplate and added the cleanup rule
    }
  }
  if (interfaces_[env_name].writing.find(type) != interfaces_[env_name].writing.end()) {
    auto &l = interfaces_[env_name].writing[type];
    auto iface_it = find_if(l.begin(), l.end(),
			    [&id] (const Interface *iface) { return id == iface->id(); });
    if (iface_it != l.end()) {
      blackboard_->close(*iface_it);
      l.erase(iface_it);
      // do NOT remove the list, even if empty, because we need to remember
      // that we already built the deftemplate and added the cleanup rule
    }
  }
}

void
BlackboardCLIPSFeature::clips_blackboard_read(std::string env_name)
{
  // no interfaces registered, that's fine
  if (interfaces_.find(env_name) == interfaces_.end())  return;
  if (envs_.find(env_name) == envs_.end()) {
    // Environment not registered, big bug
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Environment %s not registered,"
		      " cannot read interfaces", env_name.c_str());
    return;
  }

  fawkes::MutexLocker lock(envs_[env_name].objmutex_ptr());
  CLIPS::Environment &env = **(envs_[env_name]);
  for (auto &iface_map : interfaces_[env_name].reading) {
    for (auto i : iface_map.second) {
      i->read();
      if (i->changed()) {
        if (!cfg_retract_early_) {
          std::string fun = std::string("(") + i->type() + "-cleanup-late \"" + i->id() + "\")";
          env.evaluate(fun);
        }
        const Time *t = i->timestamp();

	std::string fact = std::string("(") + i->type() +
	  " (id \"" + i->id() + "\")" +
	  " (time " + StringConversions::to_string(t->get_sec()) + " "
	  + StringConversions::to_string(t->get_usec()) + ")";

	InterfaceFieldIterator f, f_end = i->fields_end();
	for (f = i->fields(); f != f_end; ++f) {
	  std::string value;
	  if (f.get_type() == IFT_BOOL) {
	    value = f.get_bool() ? "TRUE" : "FALSE";
	  } else if (f.get_type() == IFT_STRING) {
	    value = f.get_value_string();
	    std::string::size_type pos = 0;
	    while ((pos = value.find("\"", pos)) != std::string::npos) {
	      value.replace(pos, 1, "\\\"");
	      pos += 2;
	    }
	    value = std::string("\"") + value + "\"";
	  } else {
	    value = f.get_value_string();
	    std::string::size_type pos;
	    while ((pos = value.find(",")) != std::string::npos) {
	      value = value.erase(pos, 1);
	    }

	    if (f.get_type() == IFT_FLOAT || f.get_type() == IFT_DOUBLE) {
		    std::string::size_type pos;
		    while ((pos = value.find("-inf")) != std::string::npos) {
			    value = value.replace(pos, 4, std::to_string(std::numeric_limits<double>::min()));
		    }
		    while ((pos = value.find("inf")) != std::string::npos) {
			    value = value.replace(pos, 3, std::to_string(std::numeric_limits<double>::max()));
		    }
		    while ((pos = value.find("-nan")) != std::string::npos) {
			    value = value.replace(pos, 4, std::to_string(std::numeric_limits<double>::min() + 1));
		    }
		    while ((pos = value.find("nan")) != std::string::npos) {
			    value = value.replace(pos, 3, std::to_string(std::numeric_limits<double>::max() - 1));
		    }
	    }

	  }
	  fact += std::string(" (") + f.get_name() + " " + value + ")";
	}
	fact += ")";
	env.assert_fact(fact);
      }
    }
  }
}


void
BlackboardCLIPSFeature::clips_blackboard_write(std::string env_name, std::string uid)
{
  // no interfaces registered, that's fine
  if (interfaces_.find(env_name) == interfaces_.end())  return;
  if (envs_.find(env_name) == envs_.end()) {
    // Environment not registered, big bug
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Environment %s not registered,"
		      " cannot write interface %s", env_name.c_str(), uid.c_str());
    return;
  }
  std::string type, id;
  Interface::parse_uid(uid.c_str(), type, id);
  if (interfaces_[env_name].writing.find(type) != interfaces_[env_name].writing.end()) {
    auto i = std::find_if(interfaces_[env_name].writing[type].begin(),
			  interfaces_[env_name].writing[type].end(),
			  [&uid](const Interface *iface)->bool {
			    return uid == iface->uid();
			  });
    if (i != interfaces_[env_name].writing[type].end()) {
      (*i)->write();
    } else {
      logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Interface %s not opened for writing,"
			" in environment %s", uid.c_str(), env_name.c_str());
      return;
    }
  } else {
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "No interface of type %s opened for,"
		      " writing in environment %s", type.c_str(), env_name.c_str());
    return;
  }
}

void
BlackboardCLIPSFeature::clips_blackboard_get_info(std::string env_name)
{
  if (envs_.find(env_name) == envs_.end()) {
    // Environment not registered, big bug
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Environment %s not registered,"
		      " cannot read interfaces", env_name.c_str());
    return;
  }

  fawkes::LockPtr<CLIPS::Environment> &clips = envs_[env_name];

  InterfaceInfoList *iil = blackboard_->list_all();

  fawkes::MutexLocker lock(clips.objmutex_ptr());
  for (auto ii : *iil) {
    const Time *timestamp = ii.timestamp();
    std::list<std::string> quoted_readers;
    std::list<std::string> readers = ii.readers();
    std::for_each(readers.begin(), readers.end(),
		  [&quoted_readers](const std::string &r) {
		    quoted_readers.push_back(std::string("\"")+r+"\"");
		  });
    std::string quoted_readers_s = str_join(quoted_readers, ' ');
    clips->assert_fact_f("(blackboard-interface-info (id \"%s\") (type \"%s\") "
			 "(hash \"%s\") (has-writer %s) (num-readers %u) "
			 "(writer \"%s\") (readers %s) (timestamp %u %u))",
			 ii.id(), ii.type(), ii.hash_printable().c_str(),
			 ii.has_writer() ? "TRUE" : "FALSE", ii.num_readers(),
			 ii.writer().c_str(), quoted_readers_s.c_str(),
			 timestamp->get_sec(), timestamp->get_usec());
  }

  delete iil;
}


void
BlackboardCLIPSFeature::clips_blackboard_set(std::string env_name, std::string uid,
					     std::string field, CLIPS::Value value)
{
  // no interfaces registered, that's fine
  if (interfaces_.find(env_name) == interfaces_.end())  return;
  if (envs_.find(env_name) == envs_.end()) {
    // Environment not registered, big bug
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Environment %s not registered,"
		      " cannot set %s on interface %s", env_name.c_str(),
		      field.c_str(), uid.c_str());
    return;
  }
  std::string type, id;
  Interface::parse_uid(uid.c_str(), type, id);
  if (interfaces_[env_name].writing.find(type) != interfaces_[env_name].writing.end()) {
    auto i = std::find_if(interfaces_[env_name].writing[type].begin(),
			  interfaces_[env_name].writing[type].end(),
			  [&uid](const Interface *iface)->bool {
			    return uid == iface->uid();
			  });
    if (i != interfaces_[env_name].writing[type].end()) {
      set_field((*i)->fields(), (*i)->fields_end(), env_name, field, value);
    } else {
      logger_->log_error(("BBCLIPS|" + env_name).c_str(), "Interface %s not opened for writing,"
			 " in environment %s", uid.c_str(), env_name.c_str());
      return;
    }
  } else {
    logger_->log_error(("BBCLIPS|" + env_name).c_str(), "No interface of type %s opened for,"
		       " writing in environment %s", type.c_str(), env_name.c_str());
    return;
  }
}

void
BlackboardCLIPSFeature::clips_blackboard_set_multifield(std::string env_name,
							std::string uid,
							std::string field,
							CLIPS::Values values)
{
  // no interfaces registered, that's fine
  if (interfaces_.find(env_name) == interfaces_.end())  return;
  if (envs_.find(env_name) == envs_.end()) {
    // Environment not registered, big bug
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Environment %s not registered,"
		      " cannot set %s on interface %s", env_name.c_str(),
		      field.c_str(), uid.c_str());
    return;
  }
  std::string type, id;
  Interface::parse_uid(uid.c_str(), type, id);
  if (interfaces_[env_name].writing.find(type) != interfaces_[env_name].writing.end()) {
    auto i = std::find_if(interfaces_[env_name].writing[type].begin(),
			  interfaces_[env_name].writing[type].end(),
			  [&uid](const Interface *iface)->bool {
			    return uid == iface->uid();
			  });
    if (i != interfaces_[env_name].writing[type].end()) {
      set_multifield((*i)->fields(), (*i)->fields_end(), env_name, field, values);
    } else {
      logger_->log_error(("BBCLIPS|" + env_name).c_str(), "Interface %s not opened for writing,"
			 " in environment %s", uid.c_str(), env_name.c_str());
      return;
    }
  } else {
    logger_->log_error(("BBCLIPS|" + env_name).c_str(), "No interface of type %s opened for,"
		       " writing in environment %s", type.c_str(), env_name.c_str());
    return;
  }
}

CLIPS::Value
BlackboardCLIPSFeature::clips_blackboard_create_msg(std::string env_name, std::string uid,
						    std::string msg_type)
{
  // no interfaces registered, that's fine
  if (interfaces_.find(env_name) == interfaces_.end()){
    return CLIPS::Value(new std::shared_ptr<Message>());
  }
  if (envs_.find(env_name) == envs_.end()) {
    // Environment not registered, big bug
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Environment %s not registered,"
  		      " cannot read interfaces", env_name.c_str());
    return CLIPS::Value(new std::shared_ptr<Message>());
  }
  fawkes::MutexLocker lock(envs_[env_name].objmutex_ptr());
  
  std::string if_type, id;
  Interface::parse_uid(uid.c_str(), if_type, id);
  
  //get interface
  if (interfaces_[env_name].reading.find(if_type) == interfaces_[env_name].reading.end()){
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't create message for interface %s, because there is no opened interface with this type", uid.c_str());
    return CLIPS::Value(new std::shared_ptr<Message>());
  }
  auto i = std::find_if(interfaces_[env_name].reading[if_type].begin(),
			interfaces_[env_name].reading[if_type].end(),
			[&uid](const Interface *iface)->bool {
			  return uid == iface->uid();
			});
  if (i == interfaces_[env_name].reading[if_type].end()){
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't create message for interface %s, because there is no opened interface with that uid", uid.c_str());
    return CLIPS::Value(new std::shared_ptr<Message>());
  }
 
  //check if message type exists
  std::list<const char *> available_types = (*i)->get_message_types();
  bool type_exists = false;
  for(std::list<const char *>::iterator it = available_types.begin(); it != available_types.end() && !type_exists; it++){
    if(std::string(*it).compare(msg_type) == 0){
      type_exists = true;
    }
  }
  if(!type_exists){
     logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't create message for interface %s, because there is no message type %s", uid.c_str(), msg_type.c_str());
     return CLIPS::Value(new std::shared_ptr<Message>());
  }

  //create message
  Message* m = (*i)->create_message(msg_type.c_str());

  //save which interface belongs to the message
  interface_of_msg_[m] = (*i);

  //send message to clips
  return CLIPS::Value(new std::shared_ptr<Message>(m));
}

CLIPS::Values
BlackboardCLIPSFeature::clips_blackboard_list_msg_fields(std::string env_name, void *msgptr)
{
  std::shared_ptr<Message> *m =
    static_cast<std::shared_ptr<Message> *>(msgptr);
  if (!*m) {
     logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't list message fields, the pointer is wrong.");
    return CLIPS::Values();
  }

  const int field_count  = (*m)->num_fields();  
  CLIPS::Values field_names(field_count);
  int i = 0;
  for(InterfaceFieldIterator it = (*m)->fields(); it != (*m)->fields_end(); it++){
    field_names[i].set(it.get_name(), true);
    logger_->log_info(("BBCLIPS|" + env_name).c_str(), "Message has field %s", it.get_name());
    i++;
  }
  return field_names;
}


void
BlackboardCLIPSFeature::clips_blackboard_set_msg_field(std::string env_name, void *msgptr, std::string field_name, CLIPS::Value value)
{
  std::shared_ptr<Message> *m =
    static_cast<std::shared_ptr<Message> *>(msgptr);
  if (!*m) {
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't set message field, the pointer is wrong.");
    return;
  }

  bool set_success = set_field((*m)->fields(), (*m)->fields_end(),
			       env_name, field_name, value);
  if (!set_success){
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't set message field.");
  }
}


void
BlackboardCLIPSFeature::clips_blackboard_set_msg_multifield(std::string env_name, void *msgptr, std::string field_name, CLIPS::Values values)
{
  std::shared_ptr<Message> *m =
    static_cast<std::shared_ptr<Message> *>(msgptr);
  if (!*m) {
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't set message field, the pointer is wrong.");
    return;
  }

  bool set_success = set_multifield((*m)->fields(), (*m)->fields_end(),
			       env_name, field_name, values);
  if (!set_success){
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't set message field.");
  }
}


CLIPS::Value
BlackboardCLIPSFeature::clips_blackboard_send_msg(std::string env_name, void *msgptr)
{
  std::shared_ptr<Message> *m =
    static_cast<std::shared_ptr<Message> *>(msgptr);
  if (!*m) {
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't set message field, the pointer is wrong.");
    return CLIPS::Value(0);
  }
  if (!interface_of_msg_[m->get()]) {
    logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Can't send message, was it already sent?");
    return CLIPS::Value(0);
  }

  //add reference to the message so we can return the message id (otherwise it is changed by sending)
  m->get()->ref();

  unsigned int message_id = 0;

  //send message about the saved interface
  try {
	  interface_of_msg_[m->get()]->msgq_enqueue(m->get());
	  message_id = m->get()->id();
  } catch (BlackBoardNoWritingInstanceException &e) {
	  // keep quiet, BlackBoardMessageManager will already have printed a warning
	  //logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Failed to send message: no writer");
  } catch (Exception &e) {
	  logger_->log_warn(("BBCLIPS|" + env_name).c_str(), "Failed to send message: %s", e.what_no_backtrace());
  }

  //delete saved pointer to interface
  interface_of_msg_.erase(m->get());

  //remove added refference
  m->get()->unref();

  return CLIPS::Value(message_id);
}

/**
   Set array of an InterfaceFieldIterator of an Interface or Message
   to an CLIPS-Multifield.
   @return if field could successfully be set
 */
bool
BlackboardCLIPSFeature::set_multifield(InterfaceFieldIterator fit_begin,
				  InterfaceFieldIterator fit_end,
				  std::string env_name, std::string field,
				  CLIPS::Values values)
{
  //find field and check for length of the interface array/multifield
  InterfaceFieldIterator fit;
  for (fit = fit_begin; fit != fit_end; ++fit) {
    if (field == fit.get_name()) {
      size_t min_length = fit.get_length();
      if (values.size() < min_length){
	min_length = values.size();
      }
      //set each entry
      for (size_t i = 0; i < min_length; i++){
	bool success = set_field(fit, fit_end, env_name, field, values[i], i);
	if (!success){
	  return false;
	}
      }
      break;
    }
  }
  
  if (fit == fit_end) {
    logger_->log_error(("BBCLIPS|" + env_name).c_str(), "Can't find field %s",
		       field.c_str());
    return false;
  }
  return true;

}

/**
   Set field of an InterfaceFieldIterator of an Interface or Message.
   @index index in an array of the interface (leave default for non array value)
   @return if field could successfully be set
 */
bool
BlackboardCLIPSFeature::set_field(InterfaceFieldIterator fit_begin,
				  InterfaceFieldIterator fit_end,
				  std::string env_name, std::string field,
				  CLIPS::Value value, int index)
{
  InterfaceFieldIterator fit;
  for (fit = fit_begin; fit != fit_end; ++fit) {
    if (field == fit.get_name()) {
      switch (fit.get_type()) {
      case IFT_BOOL:
	if (value.type() != CLIPS::TYPE_SYMBOL && value.type() != CLIPS::TYPE_STRING) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not a symbol)",
			     field.c_str());
	  return false;
	} else {
	  std::string val_s = value.as_string();
	  if (value == "TRUE") {
	    fit.set_bool(true, index);
	  } else if (value == "FALSE") {
	    fit.set_bool(false, index);
	  } else {
	    logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			       "Cannot set field %s: invalid value %s (not a bool)",
			       field.c_str(), val_s.c_str());
	    return false;
	  }
	}
	break;

      case IFT_INT8:
	if (value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not an integer)",
			     field.c_str());
	  return false;
	} else {
	  long long int val = value.as_integer();
	  fit.set_int8((int8_t)val, index);
	}
	break;

      case IFT_UINT8:
	if (value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not an integer)",
			     field.c_str());
	  return false;
	} else {
	  long long int val = value.as_integer();
	  fit.set_uint8((uint8_t)val, index);
	}
	break;

      case IFT_INT16:
	if (value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not an integer)",
			     field.c_str());
	  return false;
	} else {
	  long long int val = value.as_integer();
	  fit.set_int16((int16_t)val, index);
	}
	break;

      case IFT_UINT16:
	if (value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not an integer)",
			     field.c_str());
	  return false;
	} else {
	  long long int val = value.as_integer();
	  fit.set_uint16((uint16_t)val, index);
	}
	break;

      case IFT_INT32:
	if (value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not an integer)",
			     field.c_str());
	  return false;
	} else {
	  long long int val = value.as_integer();
	  fit.set_int32((int32_t)val, index);
	}
	break;

      case IFT_UINT32:
	if (value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not an integer)",
			     field.c_str());
	  return false;
	} else {
	  long long int val = value.as_integer();
	  fit.set_uint32((uint32_t)val, index);
	}
	break;

      case IFT_INT64:
	if (value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not an integer)",
			     field.c_str());
	  return false;
	} else {
	  long long int val = value.as_integer();
	  fit.set_int64((int64_t)val, index);
	}
	break;

      case IFT_UINT64:
	if (value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value (not an integer)",
			     field.c_str());
	  return false;
	} else {
	  long long int val = value.as_integer();
	  fit.set_uint64((uint64_t)val, index);
	}
	break;

      case IFT_FLOAT:
	if (value.type() != CLIPS::TYPE_FLOAT && value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value "
			     "(neither float nor integer)",
			     field.c_str());
	  return false;
	} else {
	  if (value.type() == CLIPS::TYPE_FLOAT) {
	    double val = value.as_float();
	    fit.set_float((float)val, index);
	  } else {
	    long long int val = value.as_integer();
	    fit.set_float((float)val, index);
	  }
	}
	break;

      case IFT_DOUBLE:
	if (value.type() != CLIPS::TYPE_FLOAT && value.type() != CLIPS::TYPE_INTEGER) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value "
			     "(neither double nor integer)",
			     field.c_str());
	  return false;
	} else {
	  if (value.type() == CLIPS::TYPE_FLOAT) {
	    double val = value.as_float();
	    fit.set_double((double)val, index);
	  } else {
	    long long int val = value.as_integer();
	    fit.set_double((double)val, index);
	  }
	}
	break;

      case IFT_STRING:
	if (value.type() != CLIPS::TYPE_SYMBOL && value.type() != CLIPS::TYPE_STRING) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value "
			     "(neither symbol nor string)",
			     field.c_str());
	  return false;
	} else {
	  std::string val = value.as_string();
	  fit.set_string(val.c_str());
	  if (index != 0){
	    logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			       "Cannot set field %s[%d]: "
			       "there are no string arrays in interfaces",
			       field.c_str(), index);
	  }
	}
	break;

      case IFT_ENUM:
	if (value.type() != CLIPS::TYPE_SYMBOL) {
	  logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			     "Cannot set field %s: invalid value "
			     "(not a symbol)",
			     field.c_str());
	} else {
	  try {
	    std::string val = value.as_string();
	    fit.set_enum_string(val.c_str(), index);
	  } catch (Exception &e) {
	    logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			       "Failed to set enum field %s to %s, exception follows",
			       field.c_str(), value.as_string().c_str());
	    logger_->log_error(("BBCLIPS|" + env_name).c_str(), e);
	    return false;
	  }
	}
	break;

      default:
	logger_->log_error(("BBCLIPS|" + env_name).c_str(),
			   "Setting of field type %s for %s not supported",
			   fit.get_typename(), field.c_str());
	return false;
      }

      break;
    }
  }

  if (fit == fit_end) {
    logger_->log_error(("BBCLIPS|" + env_name).c_str(), "Can't find field %s",
		       field.c_str());
    return false;
  }
  return true;
}
