
/***************************************************************************
 *  config-rest-api.cpp - Configuration REST API
 *
 *  Created: Thu Apr 12 19:00:59 2018
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

#include "config-rest-api.h"

#include <config/config.h>
#include <webview/rest_api_manager.h>

#include <rapidjson/document.h>
#include <rapidjson/pointer.h>

#include <memory>

using namespace fawkes;

/** @class ConfigurationRestApi "skiller-rest-api.h"
 * REST API backend for plugins.
 * @author Tim Niemueller
 */

/** Constructor. */
ConfigurationRestApi::ConfigurationRestApi()
	: Thread("ConfigurationRestApi", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
ConfigurationRestApi::~ConfigurationRestApi()
{
}

void
ConfigurationRestApi::init()
{
	rest_api_ = new WebviewRestApi("config", logger);
	rest_api_->add_handler<ConfigTree>
		(WebRequest::METHOD_GET, "/?",
		 std::bind(&ConfigurationRestApi::cb_get_config, this, std::placeholders::_1));
	webview_rest_api_manager->register_api(rest_api_);
}

void
ConfigurationRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
}


void
ConfigurationRestApi::loop()
{
}

static rapidjson::Value
create_value(std::unique_ptr<fawkes::Configuration::ValueIterator> &i,
             rapidjson::Document::AllocatorType& a)
{
	rapidjson::Value v;
	if (i->is_list()) {
		v.SetArray();
		v.Reserve(i->get_list_size(), a);
		if (i->is_float()) {
			std::vector<float> ivs = i->get_floats();
			for (const auto &ivsv : ivs) {
				v.PushBack(rapidjson::Value(ivsv).Move(), a);
			}
		} else if (i->is_uint()) {
			std::vector<unsigned int> ivs = i->get_uints();
			for (const auto &ivsv : ivs) {
				v.PushBack(rapidjson::Value(ivsv).Move(), a);
			}
		} else if (i->is_int()) {
			std::vector<int> ivs = i->get_ints();
			for (const auto &ivsv : ivs) {
				v.PushBack(rapidjson::Value(ivsv).Move(), a);
			}
		} else if (i->is_int()) {
			std::vector<int> ivs = i->get_ints();
			for (const auto &ivsv : ivs) {
				v.PushBack(rapidjson::Value(ivsv).Move(), a);
			}
		} else if (i->is_bool()) {
			std::vector<bool> ivs = i->get_bools();
			for (const auto &ivsv : ivs) {
				v.PushBack(rapidjson::Value(ivsv).Move(), a);
			}
		} else if (i->is_string()) {
			std::vector<std::string> ivs = i->get_strings();
			for (const auto &ivsv : ivs) {
				v.PushBack(rapidjson::Value(ivsv, a).Move(), a);
			}
		}
	} else {
		if (i->is_float()) {
			v.SetFloat(i->get_float());
		} else if (i->is_uint()) {
			v.SetUint(i->get_uint());
		} else if (i->is_int()) {
			v.SetInt(i->get_int());
		} else if (i->is_bool()) {
			v.SetBool(i->get_bool());
		} else if (i->is_string()) {
			v.SetString(i->get_string(), a);
		}
	}
	return v;
}

ConfigTree
ConfigurationRestApi::cb_get_config(WebviewRestParams &params)
{
	std::string query{params.query_arg("query")};

	ConfigTree response;
	response.set_kind("ConfigTree");
	response.set_apiVersion(ConfigTree::api_version());

	std::shared_ptr<rapidjson::Document> d = std::make_shared<rapidjson::Document>();
	rapidjson::Document::AllocatorType& a = d->GetAllocator();
	d->SetObject();

	//rapidjson::Value &v{*d};
	std::unique_ptr<fawkes::Configuration::ValueIterator>
		i{config->search(query.c_str())};
	while (i->next()) {
		std::vector<std::string> path_elements{str_split(i->path(), '/')};
		rapidjson::Value::MemberIterator parent = d->MemberEnd();;
		rapidjson::Value::MemberIterator m      = d->MemberBegin();
		rapidjson::Value::MemberIterator m_end  = d->MemberEnd();

		if (path_elements.size() > 1) {
			for (size_t p = 0; p < path_elements.size() - 1; ++p) {
				m = std::find_if(m, m_end,
				                 [&path_elements,&p](const auto &v)
				                 {
					                 return path_elements[p] == v.name.GetString();
				                 });
				if (m != m_end) {
					parent = m;
				} else {
					if (parent != d->MemberEnd()) {
						parent->value.AddMember(rapidjson::Value(path_elements[p], a).Move(),
						                        rapidjson::Value(rapidjson::kObjectType).Move(), a);
						parent = parent->value.FindMember(path_elements[p].c_str());
					} else {
						d->AddMember(rapidjson::Value(path_elements[p], a).Move(),
						            rapidjson::Value(rapidjson::kObjectType).Move(), a);
						parent = d->FindMember(path_elements[p].c_str());
					}
				}
				m = parent->value.MemberBegin();
				m_end = parent->value.MemberEnd();
			}

			if (parent == d->MemberEnd()) {
				d->AddMember(rapidjson::Value(path_elements.back(), a).Move(),
				             create_value(i, a).Move(), a);
			} else {
				parent->value.AddMember(rapidjson::Value(path_elements.back(), a).Move(),
				                        create_value(i, a).Move(), a);
			}
		}
	}
	response.set_config(d);

	return response;
}
