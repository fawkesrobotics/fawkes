/***************************************************************************
 *  kdl_parser.cpp - KDL Parser
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

#include "kdl_parser.h"
#include "model.h"
#include <urdf_model/pose.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/exceptions.h>

using namespace std;
using namespace KDL;

namespace fawkes {

namespace kdl_parser {

#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif


// construct vector
Vector to_kdl(urdf::Vector3 v)
{
  return Vector(v.x, v.y, v.z);
}

// construct rotation
Rotation to_kdl(urdf::Rotation r)
{
  return Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
Frame to_kdl(urdf::Pose p)
{
  return Frame(to_kdl(p.rotation), to_kdl(p.position));
}

// construct joint
Joint to_kdl(urdf::JointSharedPtr jnt)
{
  Frame F_parent_jnt = to_kdl(jnt->parent_to_joint_origin_transform);

  switch (jnt->type){
  case urdf::Joint::FIXED:{
    return Joint(jnt->name, Joint::None);
  }
  case urdf::Joint::REVOLUTE:{
    Vector axis = to_kdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::RotAxis);
  }
  case urdf::Joint::CONTINUOUS:{
    Vector axis = to_kdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::RotAxis);
  }
  case urdf::Joint::PRISMATIC:{
    Vector axis = to_kdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::TransAxis);
  }
  default:{
    throw KDLParserUnknownJointTypeException(jnt->name.c_str());
    return Joint(jnt->name, Joint::None);
  }
  }
  return Joint();
}

// construct inertia
RigidBodyInertia to_kdl(urdf::InertialSharedPtr i)
{
  Frame origin = to_kdl(i->origin);
  // kdl specifies the inertia in the reference frame of the link, the urdf specifies the inertia in the inertia reference frame
  return origin.M * RigidBodyInertia(i->mass, origin.p, RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz));
}


// recursive function to walk through tree
bool add_children_to_tree(urdf::LinkSharedPtr root, Tree& tree)
{
  const std::vector<urdf::LinkSharedPtr > children = root->child_links;
  //ROS_DEBUG("Link %s had %i children", root->name.c_str(), (int)children.size());

  // constructs the optional inertia
  RigidBodyInertia inert(0);
  if (root->inertial)
    inert = to_kdl(root->inertial);

  // constructs the kdl joint
  Joint jnt = to_kdl(root->parent_joint);

  // construct the kdl segment
  Segment sgm(root->name, jnt, to_kdl(root->parent_joint->parent_to_joint_origin_transform), inert);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);

  // recursively add all children
  for (size_t i=0; i<children.size(); i++){
    if (!add_children_to_tree(children[i], tree))
      return false;
  }
  return true;
}


bool tree_from_file(const string& file, Tree& tree)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(file);
  return tree_from_xml(&urdf_xml, tree);
}

bool tree_from_string(const string& xml, Tree& tree)
{
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml.c_str());
  return tree_from_xml(&urdf_xml, tree);
}

bool tree_from_xml(TiXmlDocument *xml_doc, Tree& tree)
{
  urdf::Model robot_model;
  if (!robot_model.initXml(xml_doc)){
    throw KDLParserModelGenerationFailedException();
  }
  return tree_from_urdf_model(robot_model, tree);
}


bool tree_from_urdf_model(const urdf::ModelInterface& robot_model, Tree& tree)
{
  tree = Tree(robot_model.getRoot()->name);

  //  add all children
  for (size_t i=0; i<robot_model.getRoot()->child_links.size(); i++)
    if (!add_children_to_tree(robot_model.getRoot()->child_links[i], tree))
      return false;

  return true;
}

} // namespace kdl_parser

} // namespace fawkes
