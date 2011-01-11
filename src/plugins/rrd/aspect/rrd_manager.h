
/***************************************************************************
 *  rrd_manager.h - Fawkes RRD manager interface
 *
 *  Created: Fri Dec 17 00:28:14 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __PLUGINS_RRD_ASPECT_RRD_MANAGER_H_
#define __PLUGINS_RRD_ASPECT_RRD_MANAGER_H_

#include <plugins/rrd/aspect/rrd_descriptions.h>
#include <core/utils/rwlock_vector.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class RRDManager <plugins/rrd/aspect/rrd_manager.h>
 * Interface for a RRD connection creator.
 * @author Tim Niemueller
 */
class RRDManager
{
 public:
  /** Virtual empty destructor. */
  virtual ~RRDManager() {}

  /** Add RRD.
   * Add an RRD which can then be fed with data using add_data().
   * @param rrd_def RRD definition
   */
  virtual void add_rrd(RRDDefinition *rrd_def) = 0;

  /** Remove RRD.
   * Remove a RRD definition. This also removes all associated graphs.
   * @param rrd_def RRD definition
   */
  virtual void remove_rrd(RRDDefinition *rrd_def) = 0;

  /** Add graph.
   * Add a graph definition from which to generate graphs.
   * @param rrd_graph_def RRD graph definition
   */
  virtual void add_graph(RRDGraphDefinition *rrd_graph_def) = 0;

  /** Add data.
   * Add data to an RRF.
   * @param rrd_name name of the RRD to add data to
   * @param format format string. It must have the form TIMESTAMP|N:DATA,
   * where TIMESTAMP|N is either a timestamp (in seconds since the epoch), or
   * the letter N to use the current time. DATA is a concatenation of formats
   * according to man sprintf and concatenated by colons, e.g. 1:2:3:4.5.
   */
  virtual void add_data(const char *rrd_name, const char *format, ...) = 0;

  /** Get RRDs.
   * @return vector of all current RRD definitions.
   */
  virtual const RWLockVector<RRDDefinition *> &       get_rrds() const = 0;

  /** Get graphs.
   * @return vector of all current graph definitions.
   */
  virtual const RWLockVector<RRDGraphDefinition *> &  get_graphs() const = 0;

};

} // end namespace fawkes

#endif
