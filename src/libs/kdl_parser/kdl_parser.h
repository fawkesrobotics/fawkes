/***************************************************************************
 *  kdl_parser.h - KDL Parser
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

/* This code is based on ROS kdl_parser with the following copyright and license:
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

#ifndef KDL_PARSER_H
#define KDL_PARSER_H

#include <kdl/tree.hpp>
#include <string>
#include <urdf_model/model.h>
#include <tinyxml.h>

#if !defined(HAVE_URDFDOM_TYPES_H)
namespace urdf {
typedef boost::shared_ptr<urdf::Joint> JointSharedPtr;
typedef boost::shared_ptr<urdf::Inertial> InertialSharedPtr;
typedef boost::shared_ptr<urdf::Link> LinkSharedPtr;
typedef boost::shared_ptr<urdf::ModelInterface> ModelInterfaceSharedPtr;
}
#endif

namespace fawkes {

namespace kdl_parser {

#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif


/** Constructs a KDL tree from a file, given the file name
 * @param file The filename from where to read the xml
 * @param tree The resulting KDL Tree
 * @return true on success, false on failure
 */
bool tree_from_file(const std::string& file, KDL::Tree& tree);

/** Constructs a KDL tree from the parameter server, given the parameter name
 * @param param the name of the parameter on the parameter server
 * @param tree The resulting KDL Tree
 * @return true on success, false on failure
 */
bool tree_from_param(const std::string& param, KDL::Tree& tree);

/** Constructs a KDL tree from a string containing xml
 * @param xml A string containting the xml description of the robot
 * @param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
bool tree_from_string(const std::string& xml, KDL::Tree& tree);

/** Constructs a KDL tree from a TiXmlDocument
 * @param xml_doc The TiXmlDocument containting the xml description of the robot
 * @param tree The resulting KDL Tree
 * @return true on success, false on failure
 */
bool tree_from_xml(TiXmlDocument *xml_doc, KDL::Tree& tree);

/** Constructs a KDL tree from a URDF robot model
 * @param robot_model The URDF robot model
 * @param tree The resulting KDL Tree
 * @return true on success, false on failure
 */
bool tree_from_urdf_model(const urdf::ModelInterface& robot_model, KDL::Tree& tree);
}

}

#endif
