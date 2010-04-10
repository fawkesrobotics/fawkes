
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

#include <cassert>
#include <cstring>
#include <string>
#include <vector>

#include <blackboard/interface_observer.h>
#include <core/utils/lock_set.h>
#include <interfaces/ObjectPositionInterface.h>

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
  typedef fawkes::ObjectPositionInterface Opi;

  /** Wrapper that compares by the Opi's id(). */
  class OpiWrapper {
   public:
    OpiWrapper(Opi* opi) : opi_(opi) { assert(opi != NULL); }
    operator Opi*() const { return opi_; }

    bool operator == (const OpiWrapper& o) const { return cmp(o) == 0; }
    bool operator < (const OpiWrapper& o) const { return cmp(o) < 0; }

    Opi* opi() { return opi_; }
    const Opi* opi() const { return opi_; }

   private:
    int cmp(const OpiWrapper& o) const { return strcmp(opi_->id(),
                                                       o.opi_->id()); }
    Opi* opi_;
  };

  typedef fawkes::LockSet<OpiWrapper> OpiSet;
  typedef std::vector<Opi*>           OpiBucket;
  typedef std::vector<OpiBucket>      OpiBuckets;

  const static float GROUP_RADIUS = 1.0f;

  void check();
  void copy_own_if();
  void average(const OpiBucket& input_ifs);

  static float length(float x, float y, float z);
  static float rel_length(const Opi* iface);
  static float world_object_dist(const Opi* from, const Opi* to);
  static bool same_contents(const OpiBucket& left, const OpiBucket& right);

  fawkes::Logger     *logger_;
  fawkes::BlackBoard *blackboard_;

  std::string own_id_;
  std::string output_id_;

  float self_confidence_radius_;

  Opi*   own_if_;
  OpiSet input_ifs_;
  Opi*   output_if_;
};

#endif

