
/***************************************************************************
 *  pddl_robot_memory_thread.cpp - pddl_robot_memory
 *
 *  Plugin created: Thu Oct 13 13:34:05 2016

 *  Copyright  2016  Frederik Zwilling
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

#include "pddl_robot_memory_thread.h"

#include <utils/misc/string_conversions.h>

#include <bsoncxx/exception/exception.hpp>
#include <bsoncxx/json.hpp>
#include <fstream>

using namespace fawkes;
using namespace mongocxx;
using namespace bsoncxx;
using namespace bsoncxx::builder;

/** @class PddlRobotMemoryThread 'pddl_robot_memory_thread.h' 
 * Generate PDDL files from the robot memory
 *
 * This plugin uses the template engine ctemplate to generate a pddl
 * from a template file and the robot memory.
 *
 * The template file can use the following templates to generate some output
 * for each document returned by a query:
 *
 * Example: \c "<<#ONTABLE|{relation:'on-table'}>> (on-table <<object>>) <</ONTABLE>>"
 * Yields: (on-table a) (on-table b)
 * When these documents are in the database:
 *  {relation:'on-table', object:'a'}, {relation:'on-table', object:'b'}
 *
 * The selection template \c "<<#UNIQUENAME|query>> something <</UNIQUENAME>>"
 *  queries the robot memory and inserts 'something' for each returned document.
 *
 * Variable templates \c "<<key>>" inside the selection are substituted by the values
 *  of that key in the document returned by the query. You can also access subdocuments
 *  and arrays as follows:
 *  (e.g. \c "<<position_translation_0>>" for a document {position:{translation:[0,1,2], orientation:[0,1,2]}})
 *
 * @author Frederik Zwilling
 */

PddlRobotMemoryThread::PddlRobotMemoryThread()
: Thread("PddlRobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("PddlRobotMemoryThread")
{
}

void
PddlRobotMemoryThread::init()
{
	//read config values
	collection = config->get_string("plugins/pddl-robot-memory/collection");
	input_path = StringConversions::resolve_path(
	  "@BASEDIR@/src/" + config->get_string("plugins/pddl-robot-memory/input-problem-description"));
	output_path = StringConversions::resolve_path(
	  "@BASEDIR@/src/" + config->get_string("plugins/pddl-robot-memory/output-problem-description"));
	if (config->exists("plugins/pddl-robot-memory/goal"))
		goal = config->get_string("plugins/pddl-robot-memory/goal");

	//setup interface
	gen_if = blackboard->open_for_writing<PddlGenInterface>(
	  config->get_string("plugins/pddl-robot-memory/interface-name").c_str());
	gen_if->set_msg_id(0);
	gen_if->set_final(false);
	gen_if->write();

	//setup interface listener
	bbil_add_message_interface(gen_if);
	blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

	if (config->get_bool("plugins/pddl-robot-memory/generate-on-init")) {
		wakeup(); //activates loop where the generation is done
	}
}

/**
 * Thread is only waked up if there is a new interface message to generate a pddl
 */
void
PddlRobotMemoryThread::loop()
{
	//read input template of problem description
	std::string   input;
	std::ifstream istream(input_path);
	if (istream.is_open()) {
		input =
		  std::string((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());
		istream.close();
	} else {
		logger->log_error(name(), "Could not open %s", input_path.c_str());
	}
	//set template delimeters to << >>
	input = "{{=<< >>=}}" + input;

	//Dictionary how to fill the templates
	ctemplate::TemplateDictionary dict("pddl-rm");

	basic::document facets;

	//find queries in template
	size_t                             cur_pos = 0;
	std::map<std::string, std::string> templates;
	while (input.find("<<#", cur_pos) != std::string::npos) {
		// Example string with positions
		// <<#FACTS|{"relation": "domain-fact"}>> (<<name>> <<params>><</FACTS>>
		//         ^^                          ^ tpl_end_pos
		//         ||
		//         |\ q_start_pos
		//         \ q_del_pos
		// <<#FACTS||{ "$match": {"relation": "domain-fact"} }>> (<<name>> <<params>><</FACTS>>
		//         ^ ^                                        ^ tpl_end_pos
		//         | |
		//         | \ q_start_pos
		//         \ q_del_pos
		//
		//
		cur_pos                  = input.find("<<#", cur_pos) + 3;
		const size_t tpl_end_pos = input.find(">>", cur_pos);
		//is a query in the template? (indicated by '|')
		const size_t q_del_pos = input.find("|", cur_pos);
		if (q_del_pos == std::string::npos || q_del_pos > tpl_end_pos)
			continue; //no query to execute
			          //parse: template name | query
		const bool        is_simple_query = input[q_del_pos + 1] != '|';
		const size_t      q_start_pos     = q_del_pos + (is_simple_query ? 1 : 2);
		const std::string template_name   = input.substr(cur_pos, q_del_pos - cur_pos);
		const std::string query_str       = input.substr(q_start_pos, tpl_end_pos - q_start_pos);
		if (templates.find(template_name) != templates.end()) {
			if (templates[template_name] != query_str) {
				logger->log_error(name(),
				                  "Template with same name '%s' but different query '%s' vs '%s'!",
				                  template_name.c_str(),
				                  query_str.c_str(),
				                  templates[template_name].c_str());
			} else {
				input.erase(q_del_pos, tpl_end_pos - q_del_pos);
				continue;
			}
		}
		if (is_simple_query) {
			logger->log_info(name(),
			                 "Using '%s' as simple query for template '%s'",
			                 query_str.c_str(),
			                 template_name.c_str());
		} else {
			logger->log_info(name(),
			                 "Using '%s' as custom aggregate pipeline for template '%s'",
			                 query_str.c_str(),
			                 template_name.c_str());
		}
		templates[template_name] = query_str;
		//remove query stuff from input (its not part of the ctemplate features)
		input.erase(q_del_pos, tpl_end_pos - q_del_pos);

		try {
			if (is_simple_query) {
				facets.append(basic::kvp(template_name, [query_str](basic::sub_array array) {
					basic::document query;
					query.append(basic::kvp("$match", from_json(query_str)));
					array.append(query.view());
				}));
			} else {
				// We expect the query to be a complete aggregate pipeline.
				facets.append(basic::kvp(template_name, [&query_str](basic::sub_array array) {
					array.append(concatenate(from_json(query_str)));
				}));
				;
			}
		} catch (bsoncxx::exception &e) {
			logger->log_error("PddlRobotMemory",
			                  "Template query failed: %s\n%s",
			                  e.what(),
			                  query_str.c_str());
		}
	}

	mongocxx::pipeline aggregate_pipeline{};
	aggregate_pipeline.facet(facets.view());
	logger->log_info(name(),
	                 "Aggregate pipeline: %s",
	                 to_json(aggregate_pipeline.view_deprecated()).c_str());
	auto res = robot_memory->aggregate(aggregate_pipeline, collection);
	for (auto doc : res) {
		for (document::element ele : doc) {
			// check validity && type before trying to iterate
			if (ele && ele.type() == type::k_array) {
				array::view subarray{ele.get_array().value};
				for (array::element msg : subarray) {
					if (msg.type() == bsoncxx::type::k_document) {
						ctemplate::TemplateDictionary *entry_dict =
						  dict.AddSectionDictionary(std::string(ele.key()).c_str());
						fill_dict_from_document(entry_dict, msg.get_document().view());
					} else {
						throw Exception(
						  (std::string("Error while retrieving domain facts and objects: expected document "
						               "type but got ")
						   + bsoncxx::to_string(msg.type()))
						    .c_str());
					}
				}
			} else {
				throw Exception(
				  (std::string(
				     "Error while retrieving domain facts and objects: expected k_array type but got: ")
				   + bsoncxx::to_string(ele.type()))
				    .c_str());
			}
		}
	}

	//Add goal to dictionary
	dict.SetValue("GOAL", goal);

	//prepare template expanding
	ctemplate::StringToTemplateCache("tpl-cache", input, ctemplate::DO_NOT_STRIP);
	if (!ctemplate::TemplateNamelist::IsAllSyntaxOkay(ctemplate::DO_NOT_STRIP)) {
		logger->log_error(name(), "Syntax error in template %s:", input_path.c_str());
		std::vector<std::string> error_list =
		  ctemplate::TemplateNamelist::GetBadSyntaxList(false, ctemplate::DO_NOT_STRIP);
		for (std::string error : error_list) {
			logger->log_error(name(), "%s", error.c_str());
		}
	}
	//Let ctemplate expand the input
	std::string output;
	ctemplate::ExpandTemplate("tpl-cache", ctemplate::DO_NOT_STRIP, &dict, &output);

	//generate output
	logger->log_info(name(), "Output:\n%s", output.c_str());
	std::ofstream ostream(output_path);
	if (ostream.is_open()) {
		ostream << output.c_str();
		ostream.close();
	} else {
		logger->log_error(name(), "Could not open %s", output_path.c_str());
	}

	logger->log_info(name(), "Generation of PDDL problem description finished");
	gen_if->set_final(true);
	gen_if->write();
}

void
PddlRobotMemoryThread::finalize()
{
	blackboard->close(gen_if);
}

bool
PddlRobotMemoryThread::bb_interface_message_received(Interface *      interface,
                                                     fawkes::Message *message) noexcept
{
	if (message->is_of_type<PddlGenInterface::GenerateMessage>()) {
		PddlGenInterface::GenerateMessage *msg = (PddlGenInterface::GenerateMessage *)message;
		gen_if->set_msg_id(msg->id());
		gen_if->set_final(false);
		gen_if->write();
		if (std::string(msg->goal()) != "")
			goal = msg->goal();
		wakeup(); //activates loop where the generation is done
	} else {
		logger->log_error(name(), "Received unknown message of type %s, ignoring", message->type());
	}
	return false;
}

/**
 * Fills a dictionary with key value pairs from a document. Recursive to handle subdocuments
 * @param dict Dictionary to fill
 * @param obj Document
 * @param prefix Prefix of previous super-documents keys
 */
void
PddlRobotMemoryThread::fill_dict_from_document(ctemplate::TemplateDictionary *dict,
                                               const bsoncxx::document::view &doc,
                                               std::string                    prefix)
{
	for (auto elem : doc) {
		switch (elem.type()) {
		case type::k_double:
			dict->SetValue(prefix + std::string(elem.key()), std::to_string(elem.get_double()));
			break;
		case type::k_utf8:
			dict->SetValue(prefix + std::string(elem.key()), elem.get_utf8().value.to_string());
			break;
		case type::k_bool:
			dict->SetValue(prefix + std::string(elem.key()), std::to_string(elem.get_bool()));
			break;
		case type::k_int32:
			dict->SetIntValue(prefix + std::string(elem.key()), elem.get_int32());
			break;
		case type::k_int64:
			dict->SetIntValue(prefix + std::string(elem.key()), elem.get_int64());
			break;
		case type::k_document:
			fill_dict_from_document(dict,
			                        elem.get_document().view(),
			                        prefix + std::string(elem.key()) + "_");
			break;
		case type::k_oid: //ObjectId
			dict->SetValue(prefix + std::string(elem.key()), elem.get_oid().value.to_string());
			break;
		case type::k_array: {
			// access array elements as if they were a subdocument with key-value pairs
			// using the indices as keys
			basic::document b;
			array::view     array = elem.get_array().value;
			uint            i     = 0;
			for (auto e : array) {
				b.append(basic::kvp(std::to_string(i++), e.get_value()));
			}
			fill_dict_from_document(dict, b.view(), prefix + std::string(elem.key()) + "_");
			// additionally feed the whole array as space-separated list
			std::string array_string;
			for (auto e : array) {
				// TODO:adapt to other types.
				array_string += " ";
				switch (e.type()) {
				case type::k_int64: array_string += std::to_string(e.get_int64()); break;
				case type::k_utf8: array_string += e.get_utf8().value.to_string(); break;
				default: throw Exception("Not implemented");
				}
			}
			dict->SetValue(prefix + std::string(elem.key()), array_string);
			break;
		}
		default: dict->SetValue(prefix + std::string(elem.key()), "INVALID_VALUE_TYPE");
		}
	}
}
