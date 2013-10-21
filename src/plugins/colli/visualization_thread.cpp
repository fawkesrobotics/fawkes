
/***************************************************************************
 *  visualization_thread.cpp - Visualization for colli
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

#include "visualization_thread.h"

#ifdef HAVE_VISUAL_DEBUGGING

#include "common/defines.h"
#include "utils/rob/robo_laser.h"
#include "search/og_laser.h"
#include "search/astar_search.h"

#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>
#include <utils/math/types.h>

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/MarkerArray.h>

using namespace fawkes;

/** @class ColliVisualizationThread "visualization_thread.h"
 * @author Bahram Maleki-Fard
 */

/** Constructor. */
ColliVisualizationThread::ColliVisualizationThread()
 : fawkes::Thread("ColliVisualizationThread", Thread::OPMODE_WAITFORWAKEUP),
   occ_grid_( 0 ),
   search_( 0 ),
   laser_( 0 )
{
}

void
ColliVisualizationThread::init()
{
  pub_laser_ = new ros::Publisher();
  *pub_laser_ = rosnode->advertise< nav_msgs::GridCells >("colli_laser", 1);

  pub_cells_occ_ = new ros::Publisher();
  *pub_cells_occ_ = rosnode->advertise< nav_msgs::GridCells >("colli_cells_occupied", 1);

  pub_cells_near_ = new ros::Publisher();
  *pub_cells_near_ = rosnode->advertise< nav_msgs::GridCells >("colli_cells_near", 1);

  pub_cells_mid_ = new ros::Publisher();
  *pub_cells_mid_ = rosnode->advertise< nav_msgs::GridCells >("colli_cells_mid", 1);

  pub_cells_far_ = new ros::Publisher();
  *pub_cells_far_ = rosnode->advertise< nav_msgs::GridCells >("colli_cells_far", 1);

  pub_cells_free_ = new ros::Publisher();
  *pub_cells_free_ = rosnode->advertise< nav_msgs::GridCells >("colli_cells_free", 1);

  pub_search_path_ = new ros::Publisher();
  *pub_search_path_ = rosnode->advertise< nav_msgs::GridCells >("colli_search_path", 1);
}

void
ColliVisualizationThread::finalize()
{
  pub_laser_->shutdown();
  delete pub_laser_;

  pub_cells_occ_->shutdown();
  delete pub_cells_occ_;
  pub_cells_near_->shutdown();
  delete pub_cells_near_;
  pub_cells_mid_->shutdown();
  delete pub_cells_mid_;
  pub_cells_far_->shutdown();
  delete pub_cells_far_;
  pub_cells_free_->shutdown();
  delete pub_cells_free_;

  pub_search_path_->shutdown();
  delete pub_search_path_;
}


void
ColliVisualizationThread::loop()
{
  if( (occ_grid_ == NULL) || (search_ == NULL) || (laser_ == NULL) )
    return;

  MutexLocker lock(&mutex_);

  // define grid settings
  nav_msgs::GridCells grid;
  grid.header.frame_id = "/base_laser";
  grid.cell_width = 0.05;
  grid.cell_height = 0.05;

  // publish laser
  for( int i=0; i < laser_->GetNumberOfReadings(); ++i) {
    geometry_msgs::Point p;
    p.x =  laser_->GetReadingPosX(i);
    p.y =  laser_->GetReadingPosY(i);
    p.z = 0;
    grid.cells.push_back(p);
  }
  grid.header.stamp = ros::Time::now();
  pub_laser_->publish(grid);

  // publish grid cells
  grid.cells.clear();
  nav_msgs::GridCells grid_cells_occ(grid);
  nav_msgs::GridCells grid_cells_near(grid);
  nav_msgs::GridCells grid_cells_mid(grid);
  nav_msgs::GridCells grid_cells_far(grid);
  nav_msgs::GridCells grid_cells_free(grid);
  Probability prob;
  point_t gridpos_laser = occ_grid_->GetLaserPosition();
  for( int y=0; y < occ_grid_->getHeight(); ++y ) {
    for( int x=0; x < occ_grid_->getWidth(); ++x ) {
      geometry_msgs::Point p;
      p.x =  (x - gridpos_laser.x) * grid.cell_width;
      p.y =  (y - gridpos_laser.y) * grid.cell_height;
      p.z = 0;

      prob = occ_grid_->getProb(x,y);
      if( prob == _COLLI_CELL_OCCUPIED_) {
        grid_cells_occ.cells.push_back( p );

      } else if( prob == _COLLI_CELL_NEAR_ ) {
        grid_cells_near.cells.push_back( p );

      } else if( prob == _COLLI_CELL_MIDDLE_ ) {
        grid_cells_mid.cells.push_back( p );

      } else if( prob == _COLLI_CELL_FAR_ ) {
        grid_cells_far.cells.push_back( p );

      } else if( prob == _COLLI_CELL_FREE_ ) {
        grid_cells_free.cells.push_back( p );
      }
    }
  }
  pub_cells_occ_->publish( grid_cells_occ );
  pub_cells_near_->publish( grid_cells_near );
  pub_cells_mid_->publish( grid_cells_mid );
  pub_cells_far_->publish( grid_cells_far );
  pub_cells_free_->publish( grid_cells_free );

  // publish path
  grid.cells.clear();
  std::vector< point_t >* plan = search_->GetPlan();
  point_t gridpos_robo = search_->GetRoboPosition();
  for( std::vector<point_t>::iterator it=plan->begin(); it!=plan->end(); ++it ) {
    geometry_msgs::Point p;
    p.x =  ((*it).x - gridpos_robo.x) * grid.cell_width;
    p.y =  ((*it).y - gridpos_robo.y) * grid.cell_height;
    p.z = 0;
    grid.cells.push_back( p );
  }
  pub_search_path_->publish( grid );

}

void
ColliVisualizationThread::setup(CLaserOccupancyGrid* occ_grid,
                               CSearch* search,
                               Laser* laser)
{
  MutexLocker lock(&mutex_);
  occ_grid_ = occ_grid;
  search_   = search;
  laser_    = laser;
}

#endif
