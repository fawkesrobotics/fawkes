
/***************************************************************************
 *  objpos_majority.h - Fawkes WorldModel Object Position Majority Fuser
 *
 *  Created: Thu 01 Apr 2010 05:06:36 PM CEST
 *  Copyright  2010  Christoph Schwering
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

#ifndef __PLUGINS_WORLDMODEL_FUSER_OBJPOS_MAJORITY_H_
#define __PLUGINS_WORLDMODEL_FUSER_OBJPOS_MAJORITY_H_

#include <string>
#include <vector>

#include <blackboard/interface_observer.h>
#include <core/utils/lock_list.h>

#include "fuser.h"

namespace fawkes
{
  class BlackBoard;
  class Logger;
  class ObjectPositionInterface;
}

class WorldModelObjPosMajorityFuser
: public WorldModelFuser,
  public fawkes::BlackBoardInterfaceObserver
{
 public:
  WorldModelObjPosMajorityFuser(fawkes::Logger* logger,
                                fawkes::BlackBoard* blackboard,
                                const std::string& own_id,
                                const std::string& foreign_id_pattern,
                                const std::string& output_id,
                                float self_confidence_radius);
  ~WorldModelObjPosMajorityFuser();

  virtual void bb_interface_created(const char *type, const char *id) throw();
  virtual void fuse();

 private:
  typedef fawkes::ObjectPositionInterface OPI;
  typedef fawkes::LockList<OPI*>          OPIList;
  typedef std::vector<OPI*>               OPIBucket;
  typedef std::vector<OPIBucket>          OPIBuckets;

  const static float GROUP_RADIUS = 1.0f;

  void check();
  void copy_own_if();
  void average(const OPIBucket& input_ifs);

  static float length(float x, float y, float z);
  static float rel_length(const OPI* iface);
  static float world_dist(const OPI* from, const OPI* to);

  fawkes::BlackBoard *blackboard_;
  fawkes::Logger     *logger_;

  std::string own_id_;
  std::string output_id_;

  float self_confidence_radius_;

  OPI*     own_if_;
  OPIList  input_ifs_;
  OPI*     output_if_;
};

#endif

