
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
#include <fstream>
#include <utils/misc/string_conversions.h>

using namespace fawkes;
using namespace mongo;

/** @class PddlRobotMemoryThread 'pddl_robot_memory_thread.h' 
 * Generate PDDL files from the robot memory
 *
 * This plugin uses the template engine ctemplate to generate a pddl
 * from a template file and the robot memory.
 *
 * The template file can use the following templates to generate some output
 * for each document returned by a query:
 *
 * Example: <<#ONTABLE|{relation:'on-table'}>> (on-table <<object>>) <</ONTABLE>>
 * Yields: (on-table a) (on-table b)
 * When these documents are in the database:
 *  {relation:'on-table', object:'a'}, {relation:'on-table', object:'b'}
 *
 * The selection template <<#UNIQUENAME|query>> something <</UNIQUENAME>>
 *  queries the robot memory and inserts 'something' for each returned document.
 *
 * Variable templates <<key>> inside the selection are substituted by the values
 *  of that key in the document returned by the query. You can also access subdocuments
 *  (e.g. <<position.orientation.yaw>> )
 *
 * @author Frederik Zwilling
 */

PddlRobotMemoryThread::PddlRobotMemoryThread()
 : Thread("PddlRobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
             BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL) 
{
}

void
PddlRobotMemoryThread::init()
{
  //read config values
  collection = config->get_string("plugins/pddl-robot-memory/collection");
  input_path = StringConversions::resolve_path("@BASEDIR@/src/agents/" +
    config->get_string("plugins/pddl-robot-memory/input-problem-description"));
  output_path = StringConversions::resolve_path("@BASEDIR@/src/agents/" +
    config->get_string("plugins/pddl-robot-memory/output-problem-description"));

  //read input template of problem description
  std::string input;
  std::ifstream istream(input_path);
  if(istream.is_open())
  {
    input = std::string((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());
    istream.close();
  }
  else
  {
    logger->log_error(name(), "Could not open %s", input_path.c_str());
  }
  //set template delimeters to << >>
  input = "{{=<< >>=}}" +  input;

  //Dictionary how to fill the templates
  ctemplate::TemplateDictionary dict("pddl-rm");

  //find queries in template
  size_t cur_pos = 0;
  while(input.find("<<#", cur_pos) != std::string::npos)
  {
    cur_pos = input.find("<<#", cur_pos) + 3;
    size_t tpl_end_pos =  input.find(">>", cur_pos);
    //is a query in the template? (indicated by '|')
    size_t q_del_pos = input.find("|", cur_pos);
    if(q_del_pos == std::string::npos || q_del_pos > tpl_end_pos)
      continue; //no query to execute
    //parse: template name | query
    std::string template_name = input.substr(cur_pos, q_del_pos - cur_pos);
    std::string query_str =  input.substr(q_del_pos + 1, tpl_end_pos - (q_del_pos + 1));
    //remove query stuff from input (its not part of the ctemplate features)
    input.erase(q_del_pos, tpl_end_pos - q_del_pos);

    //fill dictionary to expand query template:
    QResCursor cursor = robot_memory->query(fromjson(query_str), collection);
    while(cursor->more())
    {
      BSONObj obj = cursor->next();
      //dictionary for one entry
      ctemplate::TemplateDictionary *entry_dict = dict.AddSectionDictionary(template_name);
      fill_dict_from_document(entry_dict, obj);
    }
  }

  //prepare template expanding
  ctemplate::StringToTemplateCache("tpl-cache", input, ctemplate::DO_NOT_STRIP);
  if(!ctemplate::TemplateNamelist::IsAllSyntaxOkay(ctemplate::DO_NOT_STRIP))
  {
    logger->log_error(name(), "Syntax error in template %s:", input_path.c_str());
    std::vector<std::string> error_list = ctemplate::TemplateNamelist::GetBadSyntaxList(false, ctemplate::DO_NOT_STRIP);
    for(std::string error : error_list)
    {
      logger->log_error(name(), "", error.c_str());
    }
  }
  //Let ctemplate expand the input
  std::string output;
  ctemplate::ExpandTemplate("tpl-cache", ctemplate::DO_NOT_STRIP, &dict, &output);

  //generate output
  logger->log_info(name(), "Output:\n%s", output.c_str());
  std::ofstream ostream(output_path);
  if(ostream.is_open())
  {
    ostream << output.c_str();
    ostream.close();
  }
  else
  {
    logger->log_error(name(), "Could not open %s", output_path.c_str());
  }

  logger->log_info(name(), "Generation of PDDL problem description finished");
}

void
PddlRobotMemoryThread::loop()
{
}

void
PddlRobotMemoryThread::finalize()
{
}

/**
 * Fills a dictionary with key value pairs from a document. Recursive to handle subdocuments
 * @param dict Dictionary to fill
 * @param obj Document
 * @param prefix Prefix of previous super-documents keys
 */
void PddlRobotMemoryThread::fill_dict_from_document(ctemplate::TemplateDictionary *dict, mongo::BSONObj obj, std::string prefix)
{
  for(BSONObjIterator it = obj.begin(); it.more();)
  {
    BSONElement elem = it.next();
    switch (elem.type()) {
      case mongo::NumberDouble:
        dict->SetValue(prefix + elem.fieldName(), std::to_string(elem.Double()));
        break;
      case mongo::String:
        dict->SetValue(prefix + elem.fieldName(), elem.String());
        break;
      case mongo::Bool:
        dict->SetValue(prefix + elem.fieldName(), std::to_string(elem.Bool()));
        break;
      case mongo::NumberInt:
        dict->SetIntValue(prefix + elem.fieldName(), elem.Int());
        break;
      case mongo::NumberLong:
        dict->SetValue(prefix + elem.fieldName(), std::to_string(elem.Long()));
        break;
      case mongo::Object:
        fill_dict_from_document(dict, elem.Obj(), prefix + elem.fieldName() + "_");
        break;
      case 7: //ObjectId
      dict->SetValue(prefix + elem.fieldName(), elem.OID().toString());
      break;
      //TODO: array

      default:
        dict->SetValue(prefix + elem.fieldName(), "INVALID_VALUE_TYPE");
    }
  }
}

