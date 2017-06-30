/***************************************************************************
 *  model.cpp - URDF Model
 *
 *  Created: Fri Feb 14 17:35:15 2014
 *  Copyright  2014 Till Hofmann
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

/* This code is based on ROS robot_model with the following copyright and license:
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#include "model.h"
#include <urdf_parser/urdf_parser.h>

#include <vector>
#include <fstream>
#include <iostream>

#include <core/exceptions/system.h>
#include <kdl_parser/exceptions.h>

using namespace fawkes;

namespace urdf {

static bool IsColladaData(const std::string& data)
{
  return data.find("<COLLADA") != std::string::npos;
}


/** @class Model <kdl_parser/model.h>
 * This class represents an URDF model. It can be initialized
 * using either Strings or Files, which are parsed and saved in
 * the Model objects.
 * This class implements the ModelInterface defined in
 * /usr/include/urdf/urdf_model/model.h
 */

/** Initialize the Model using a URDF file
 * @param filename The filename of the URDF file
 * @return true if the model was intialized successfully
 */
bool Model::initFile(const std::string& filename)
{

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    return Model::initString(xml_string);
  }
  else
  {
    throw CouldNotOpenFileException(filename.c_str());
  }

}

/** Initialize the model using a XML Document
 * @param xml_doc The robot model as TiXmlDocument
 * @return true if the model was intialized successfully
 */
bool Model::initXml(TiXmlDocument *xml_doc)
{
  if (!xml_doc)
  {
    throw URDFXMLDocumentParseErrorException();
  }

  std::stringstream ss;
  ss << *xml_doc;

  return Model::initString(ss.str());
}

/** Initialize the model using a XML Element
 * @param robot_xml The robot model as TiXmlElement
 * @return true if the model was intialized successfully
 */
bool Model::initXml(TiXmlElement *robot_xml)
{
  if (!robot_xml)
  {
    throw URDFXMLElementParseErrorException();
  }

  std::stringstream ss;
  ss << (*robot_xml);

  return Model::initString(ss.str());
}


/** Initialize the model using an URDF string
 * @param xml_string The robot description in URDF format
 * @return true if the model was intialized successfully
 */
bool Model::initString(const std::string& xml_string)
{
  ModelInterfaceSharedPtr model;

  if( IsColladaData(xml_string) ) {
    // currently, support for Collada is not implemented
    throw URDFColladaNotSupportedException();
  }
  else {
    model = parseURDF(xml_string);
  }

  // copy data from model into this object
  if (model){
    this->links_ = model->links_;
    this->joints_ = model->joints_;
    this->materials_ = model->materials_;
    this->name_ = model->name_;
    this->root_link_ = model->root_link_;
    return true;
  }
  else
    return false;
}


}// namespace urdf
