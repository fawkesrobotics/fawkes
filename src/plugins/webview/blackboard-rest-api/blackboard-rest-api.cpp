
/***************************************************************************
 *  blackboard-rest-api.cpp -  Blackboard REST API
 *
 *  Created: Mon Mar 26 23:27:42 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "blackboard-rest-api.h"

#include <webview/rest_api_manager.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/wait.h>

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/interface_info.h>

using namespace fawkes;

/** @class BlackboardRestApi "skiller-rest-api.h"
 * REST API backend for the blackboard.
 * @author Tim Niemueller
 */

/** Constructor. */
BlackboardRestApi::BlackboardRestApi()
	: Thread("BlackboardRestApi", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
BlackboardRestApi::~BlackboardRestApi()
{
}

void
BlackboardRestApi::init()
{
	rest_api_ = new WebviewRestApi("blackboard", logger);
	rest_api_->add_handler<WebviewRestArray<::InterfaceInfo>>
		(WebRequest::METHOD_GET, "interfaces",
		 std::bind(&BlackboardRestApi::cb_list_interfaces, this, std::placeholders::_1));
	webview_rest_api_manager->register_api(rest_api_);
}

void
BlackboardRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
}


void
BlackboardRestApi::loop()
{
}

std::vector<std::shared_ptr<InterfaceFieldType>>
BlackboardRestApi::gen_fields(fawkes::InterfaceFieldIterator begin,
                              fawkes::InterfaceFieldIterator end)
{
	std::vector<std::shared_ptr<InterfaceFieldType>> rv;
	for (auto i = begin; i  != end; ++i) {
		std::shared_ptr<InterfaceFieldType> ft = std::make_shared<InterfaceFieldType>();
		ft->set_name(i.get_name());
		ft->set_type(i.get_typename());
		ft->set_is_array(i.get_type() != IFT_STRING && i.get_length() > 1);
		if (i.is_enum()) {
			std::list<const char *> enum_values = i.get_enum_valuenames();
			ft->set_enums(std::vector<std::string>{ std::begin(enum_values), std::end(enum_values) });
		}
		rv.push_back(ft);
	}
	return rv;
}

WebviewRestArray<::InterfaceInfo>
BlackboardRestApi::cb_list_interfaces(WebviewRestParams& params)
{
	if (params.query_arg("pretty") == "true") {
		params.set_pretty_json(true);
	}

	WebviewRestArray<::InterfaceInfo> rv;

	std::unique_ptr<InterfaceInfoList> ifls{blackboard->list_all()};

	for (const auto &ii : *ifls) {
		::InterfaceInfo info;
		info.set_kind("InterfaceInfo");
		info.set_apiVersion(::InterfaceInfo::api_version());
		info.set_id(ii.id());
		info.set_type(ii.type());
		info.set_hash(ii.hash_printable());
		if (ii.has_writer()) {
			info.set_writer(ii.writer());
		}
		std::list<std::string> readers = ii.readers();
		info.set_readers(std::vector<std::string>{ std::begin(readers), std::end(readers) });

		if (type_info_cache_.find(ii.type()) != type_info_cache_.end()) {
			info.set_fields(type_info_cache_[ii.type()].first);
			info.set_message_types(type_info_cache_[ii.type()].second);
		} else {
			Interface *iface = blackboard->open_for_reading(ii.type(), ii.id());

			std::vector<std::shared_ptr<InterfaceFieldType>> fields =
				gen_fields(iface->fields(), iface->fields_end());

			info.set_fields(fields);

			std::vector<std::shared_ptr<InterfaceMessageType>> message_types;
				
			std::list<const char *> message_type_names = iface->get_message_types();
			for (auto mt : message_type_names) {
				Message *msg = iface->create_message(mt);
				std::shared_ptr<InterfaceMessageType> m = std::make_shared<InterfaceMessageType>();
				m->set_name(mt);
				m->set_fields(gen_fields(msg->fields(), msg->fields_end()));
				message_types.push_back(m);
				delete msg;
			}
			blackboard->close(iface);
			
			info.set_message_types(message_types);
			type_info_cache_[ii.type()] = std::make_pair(fields, message_types);
		}

		rv.push_back(info);
	}

	return rv;
}
