
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

#include <rapidjson/document.h>

#include <set>

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
		(WebRequest::METHOD_GET, "/interfaces",
		 std::bind(&BlackboardRestApi::cb_list_interfaces, this));
	rest_api_->add_handler<InterfaceData>
		(WebRequest::METHOD_GET, "/interfaces/{type}/{id+}/data",
		 std::bind(&BlackboardRestApi::cb_get_interface_data, this, std::placeholders::_1));
	rest_api_->add_handler<::InterfaceInfo>
		(WebRequest::METHOD_GET, "/interfaces/{type}/{id+}",
		 std::bind(&BlackboardRestApi::cb_get_interface_info, this, std::placeholders::_1));
	rest_api_->add_handler<BlackboardGraph>
		(WebRequest::METHOD_GET, "/graph",
		 std::bind(&BlackboardRestApi::cb_get_graph, this));
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

::InterfaceInfo
BlackboardRestApi::gen_interface_info(const fawkes::InterfaceInfo &ii)
{
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
		iface->read();

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
	return info;
}

#define FIELD_ARRAY_CASE(TYPE, type, ctype)	                           \
	case IFT_ ## TYPE : {                                                \
		const ctype *values = i.get_ ## type ## s();                       \
		for (unsigned int j = 0; j < i.get_length(); ++j) {                \
			value.PushBack(rapidjson::Value{values[j]}.Move(), allocator);   \
		}                                                                  \
		break;                                                             \
	}
	

static rapidjson::Value
gen_field_value(fawkes::InterfaceFieldIterator& i,
                fawkes::Interface *iface,
                rapidjson::Document::AllocatorType& allocator)
{
	rapidjson::Value value;

	if (i.get_length() > 1) {
		if (i.get_type() == IFT_STRING) {
			value.SetString(std::string(i.get_string()), allocator);
		} else {
			value.SetArray();
			value.Reserve(i.get_length(), allocator);

			switch (i.get_type()) {
				FIELD_ARRAY_CASE(BOOL, bool, bool);
				FIELD_ARRAY_CASE(INT8, int8, int8_t);
				FIELD_ARRAY_CASE(UINT8, uint8, uint8_t);
				FIELD_ARRAY_CASE(INT16, int16, int16_t);
				FIELD_ARRAY_CASE(UINT16, uint16, uint16_t);
				FIELD_ARRAY_CASE(INT32, int32, int32_t);
				FIELD_ARRAY_CASE(UINT32, uint32, uint32_t);
				FIELD_ARRAY_CASE(INT64, int64, int64_t);
				FIELD_ARRAY_CASE(UINT64, uint64, uint64_t);
				FIELD_ARRAY_CASE(FLOAT, float, float);
				FIELD_ARRAY_CASE(DOUBLE, double, double);
				FIELD_ARRAY_CASE(BYTE, byte, uint8_t);

			case IFT_ENUM : {
				const int32_t *values = i.get_enums();
				for (unsigned int j = 0; j < i.get_length(); ++j) {
					value.PushBack(rapidjson::Value{iface->enum_tostring(i.get_typename(), values[j]), allocator}.Move(), allocator);
				}
				break;
			}

			case IFT_STRING: break; // handled above
			}
		}
	} else {
		switch (i.get_type()) {
		case IFT_BOOL:   value.SetBool(i.get_bool());	    break;
		case IFT_INT8:   value.SetInt(i.get_int8());      break;
		case IFT_UINT8:  value.SetUint(i.get_uint8());    break;
		case IFT_INT16:  value.SetInt(i.get_int16());     break;
		case IFT_UINT16: value.SetUint(i.get_uint16());   break;
		case IFT_INT32:  value.SetInt(i.get_int32());     break;
		case IFT_UINT32: value.SetUint(i.get_uint32());   break;
		case IFT_INT64:  value.SetInt64(i.get_int64());   break;
		case IFT_UINT64: value.SetUint64(i.get_uint64()); break;
		case IFT_FLOAT:  value.SetFloat(i.get_float());   break;
		case IFT_DOUBLE: value.SetDouble(i.get_double()); break;
		case IFT_BYTE:   value.SetUint(i.get_byte());     break;
		case IFT_STRING:
			[[fallthrough]]
		case IFT_ENUM:
			value.SetString(std::string(i.get_value_string()), allocator);
		}
	}
	return value;
}

InterfaceData
BlackboardRestApi::gen_interface_data(Interface *iface, bool pretty)
{
	InterfaceData data;
	data.set_kind("InterfaceData");
	data.set_apiVersion(InterfaceData::api_version());
	data.set_type(iface->type());
	data.set_id(iface->id());
	if (iface->has_writer()) {
		data.set_writer(iface->writer());
	}
	std::list<std::string> readers = iface->readers();
	data.set_readers(std::vector<std::string>{ std::begin(readers), std::end(readers) });
	data.set_timestamp(iface->timestamp()->str());

	// Generate data as JSON document
	std::shared_ptr<rapidjson::Document> d = std::make_shared<rapidjson::Document>();
	rapidjson::Document::AllocatorType& allocator = d->GetAllocator();
	d->SetObject();

	for (auto i = iface->fields(); i  != iface->fields_end(); ++i) {
		rapidjson::Value name(i.get_name(), allocator);
		d->AddMember(name.Move(), gen_field_value(i, iface, allocator).Move(), allocator);
	}
	data.set_data(d);

	return data;
}


WebviewRestArray<::InterfaceInfo>
BlackboardRestApi::cb_list_interfaces()
{
	WebviewRestArray<::InterfaceInfo> rv;

	std::unique_ptr<InterfaceInfoList> ifls{blackboard->list_all()};

	for (const auto &ii : *ifls) {
		rv.push_back(gen_interface_info(ii));
	}

	return rv;
}

::InterfaceInfo
BlackboardRestApi::cb_get_interface_info(WebviewRestParams& params)
{
	if (params.path_arg("type").find_first_of("*?") != std::string::npos) {
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST, "Type may not contain any of [*?].");
	}
	if (params.path_arg("id").find_first_of("*?") != std::string::npos) {
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST, "ID may not contain any of [*?].");
	}

	std::unique_ptr<InterfaceInfoList> ifls
		{blackboard->list(params.path_arg("type").c_str(), params.path_arg("id").c_str())};

	if (ifls->size() < 1) {
		throw WebviewRestException(WebReply::HTTP_NOT_FOUND, "Interface %s:%s not found",
		                           params.path_arg("type").c_str(), params.path_arg("id").c_str());
	}
	return gen_interface_info(ifls->front());
}

InterfaceData
BlackboardRestApi::cb_get_interface_data(WebviewRestParams& params)
{
	bool pretty = params.has_query_arg("pretty");
	params.set_pretty_json(pretty);

	if (params.path_arg("type").find_first_of("*?") != std::string::npos) {
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST, "Type may not contain any of [*?].");
	}
	if (params.path_arg("id").find_first_of("*?") != std::string::npos) {
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST, "ID may not contain any of [*?].");
	}

	std::unique_ptr<InterfaceInfoList> ifls
	{blackboard->list(params.path_arg("type").c_str(), params.path_arg("id").c_str())};
	if (ifls->size() == 0) {
		throw WebviewRestException(WebReply::HTTP_NOT_FOUND,
		                           "Interface %s::%s: is currently not available",
		                           params.path_arg("type").c_str(), params.path_arg("id").c_str());
	}

	Interface *iface = nullptr;
	try {
		iface = blackboard->open_for_reading(params.path_arg("type").c_str(),
		                                     params.path_arg("id").c_str());
		iface->read();
	} catch (Exception &e) {
		throw WebviewRestException(WebReply::HTTP_NOT_FOUND, "Failed to open %s::%s: %s",
		                           params.path_arg("type").c_str(), params.path_arg("id").c_str(),
		                           e.what_no_backtrace());
	}

	try {
		InterfaceData d{gen_interface_data(iface, pretty)};
		blackboard->close(iface);
		return d;
	} catch (Exception &e) {
		blackboard->close(iface);
		throw WebviewRestException(WebReply::HTTP_NOT_FOUND, "Failed to read %s:%s: %s",
		                           params.path_arg("type").c_str(), params.path_arg("id").c_str(),
		                           e.what_no_backtrace());
	}
}


std::string
BlackboardRestApi::generate_graph(std::string for_owner)
{
	InterfaceInfoList *iil = blackboard->list_all();
	iil->sort();

	std::stringstream mstream;
	mstream << "digraph bbmap {" << std::endl
	        << "  graph [fontsize=12,rankdir=LR];" << std::endl;

	std::set<std::string> owners;

	InterfaceInfoList::iterator ii;
	for (ii = iil->begin(); ii != iil->end(); ++ii) {
		const std::list<std::string> readers = ii->readers();
    
		if (for_owner == "" ||
		    ii->writer() == for_owner ||
		    std::find_if(readers.begin(), readers.end(),
		                 [&for_owner](const std::string &o)->bool { return for_owner == o; })
		    != readers.end())
		{
			if (ii->has_writer()) {
				const std::string writer = ii->writer();
				if (! writer.empty())  owners.insert(writer);
			}
			std::list<std::string>::const_iterator r;
			for (r = readers.begin(); r != readers.end(); ++r) {
				owners.insert(*r);
			}
		}
	}

	mstream << "  node [fontsize=12 shape=box width=4 margin=0.05];" << std::endl
	        << "  { rank=same; " << std::endl;
	std::set<std::string>::iterator i;
	for (ii = iil->begin(); ii != iil->end(); ++ii) {
		const std::list<std::string> readers = ii->readers();
		if (for_owner == "" ||
		    ii->writer() == for_owner ||
		    std::find_if(readers.begin(), readers.end(),
		                 [&for_owner](const std::string &o)->bool { return for_owner == o; })
		    != readers.end())
		{
			mstream << "    \"" << ii->type() << "::" << ii->id() << "\""
			        << " [href=\"/blackboard/view/" << ii->type() << "::" << ii->id() << "\"";

			if (! ii->has_writer()) {
				mstream << " color=red";
			} else if (ii->writer().empty()) {
				mstream << " color=purple";
			}
			mstream << "];" << std::endl;
		}
	}
	mstream << "  }" << std::endl;

	mstream << "  node [fontsize=12 shape=octagon width=3];" << std::endl;
	for (i = owners.begin(); i != owners.end(); ++i) {
		mstream << "  \"" << *i << "\""
		        << " [href=\"/blackboard/graph/" << *i << "\"];"
		        << std::endl;
	}

	for (ii = iil->begin(); ii != iil->end(); ++ii) {
		const std::list<std::string> readers = ii->readers();
		if (for_owner == "" ||
		    ii->writer() == for_owner ||
		    std::find_if(readers.begin(), readers.end(),
		                 [&for_owner](const std::string &o)->bool { return for_owner == o; })
		    != readers.end())
		{
			std::list<std::string> quoted_readers;
			std::for_each(readers.begin(), readers.end(),
			              [&quoted_readers](const std::string &r) {
				              quoted_readers.push_back(std::string("\"")+r+"\"");
			              });
			std::string quoted_readers_s = str_join(quoted_readers, ' ');
			mstream << "  \"" << ii->type() << "::" << ii->id() << "\" -> { "
			        << quoted_readers_s << " } [style=dashed arrowhead=dot arrowsize=0.5 dir=both];" << std::endl;

			if (ii->has_writer()) {
				mstream << "  \"" << (ii->writer().empty() ? "???" : ii->writer()) << "\" -> \""
				        << ii->type() << "::" << ii->id() << "\""
				        << (ii->writer().empty() ? " [color=purple]" : " [color=\"#008800\"]")
				        << ";" << std::endl;
			}
		}
	}

	delete iil;

	mstream << "}";
	return mstream.str();
}


BlackboardGraph
BlackboardRestApi::cb_get_graph()
{
	try {
		BlackboardGraph graph;
		graph.set_kind("TransformsGraph");
		graph.set_apiVersion(BlackboardGraph::api_version());
		graph.set_dotgraph(generate_graph());
		return graph;
	} catch (Exception &e) {
		throw WebviewRestException(WebReply::HTTP_INTERNAL_SERVER_ERROR,
		                           "Failed to retrieve blackboard graph: %s",
		                           e.what_no_backtrace());
	}
}
