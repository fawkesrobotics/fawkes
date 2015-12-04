
/***************************************************************************
 *  visualization_thread.h - Visualization for colli
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_VISUALIZATION_THREAD_H_
#define __PLUGINS_COLLI_VISUALIZATION_THREAD_H_

#ifdef HAVE_VISUAL_DEBUGGING

#include "common/types.h"

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/tf.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <plugins/ros/aspect/ros.h>

#include <vector>

namespace ros {
  class Publisher;
}

namespace fawkes {
  class LaserOccupancyGrid;
  class Search;
  class RoboShapeColli;
  typedef struct point_struct point_t;
}

class ColliVisualizationThread
: public fawkes::Thread,
  public fawkes::TransformAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::ROSAspect
{
 public:
  ColliVisualizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void setup(fawkes::LaserOccupancyGrid* occ_grid,
                     fawkes::Search* search);

 private:
  fawkes::Mutex mutex_;

  fawkes::LaserOccupancyGrid *occ_grid_;
  fawkes::Search             *search_;
  fawkes::RoboShapeColli    *roboshape_;
  fawkes::colli_cell_cost_t   cell_costs_;

  ros::Publisher *pub_roboshape_;

  ros::Publisher *pub_cells_occ_;
  ros::Publisher *pub_cells_near_;
  ros::Publisher *pub_cells_mid_;
  ros::Publisher *pub_cells_far_;
  ros::Publisher *pub_cells_free_;
  ros::Publisher *pub_search_path_;

  std::vector<fawkes::point_t> cells_occ_;
  std::vector<fawkes::point_t> cells_near_;
  std::vector<fawkes::point_t> cells_mid_;
  std::vector<fawkes::point_t> cells_far_;
  std::vector<fawkes::point_t> cells_free_;

  std::string frame_base_;
  std::string frame_laser_;
};

#endif
#endif
