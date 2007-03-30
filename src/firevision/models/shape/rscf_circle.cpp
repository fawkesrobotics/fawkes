
/***************************************************************************
 *  rscf_circle.cpp - Implementation of a circle shape finder
 *
 *  Generated: Fri Sep 09 2005 11:46:42
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *                   Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <cmath>
#include <sys/time.h>
#include "models/shape/rscf_circle.h"

using namespace std;

#define TEST_IF_IS_A_PIXEL(x) ((x)>230)

const int   RscfCircleModel::MAX_NUM_ITERATION  = 600;
const float RscfCircleModel::NEAR_DISTANCE      = 0.3f;
const float RscfCircleModel::MIN_RADIUS         = 10.0f;
const float RscfCircleModel::MAX_RADIUS         = 600.0f;
                                                                                                                                              


/** @class RscfCircleModel <models/shape/rht_circle.h>
 * Randomized Stable Circle Fitting Algorithm.
 */

/** Constructor. */
RscfCircleModel::RscfCircleModel()
{
}

/** Destructor. */
RscfCircleModel::~RscfCircleModel()
{
  circles.clear();
}

int
RscfCircleModel::parseImage( unsigned char* buf,
			     ROI *roi )
{
  
  unsigned char *buffer     = roi->getROIBufferStart(buf);
  unsigned char *line_start = buffer;

  unsigned int num_circles  = 0;

  unsigned int x, y;

  vector<point_t> free_points;


  free_points.clear();
  edge_pixels.clear();
  circles.clear();


  // push edge pixels to vector
  for (y = 0; y < roi->height; ++y) {
    for (x = 0; x < roi->width; ++x) {
      if (TEST_IF_IS_A_PIXEL(*buffer)) {
	point_t pt={x, y};
	edge_pixels.push_back(pt);
      }
      // NOTE: this assumes roi->pixel_step == 1
      ++buffer;
    }
    line_start += roi->line_step;
    buffer = line_start;
  }

  //  buffer     = roi->getROIBufferStart( buf );
  //  line_start = buffer;


  for (int i = 0; i<MAX_NUM_ITERATION; ++i)
    {
      // pick random point
      int ri = (int) (edge_pixels.size() * rand() / RAND_MAX);
      epit = edge_pixels.begin() + ri;
      point_t random_pixel = *epit;
      edge_pixels.erase(epit);

      if (circles.size() == 0)
	{
	  free_points.push_back(random_pixel);
	  if (free_points.size() == 3)
	    {
	      FittedCircle c;
	      c.addPoint(free_points[0]);
	      c.addPoint(free_points[1]);
	      c.addPoint(free_points[2]);
	      free_points.clear();
	      circles.push_back(c);
	    }
	  continue;
	}

      vector<FittedCircle>::iterator min_cit = circles.begin();
      float min_dist = min_cit->addPoint(random_pixel);

      // Iterate over all existing circles
      for (cit = min_cit+1; cit != circles.end(); ++cit) {
	float dist = cit->addPoint(random_pixel);
	if (dist < min_dist)
	  {
	    min_dist = dist;
	    min_cit = cit;
	  }
      }
      if (min_dist < NEAR_DISTANCE)
	{
	  min_cit->commit();
	}
      else
	{
	  free_points.push_back(random_pixel);
	  if (free_points.size() == 3)
	    {
	      FittedCircle c;
	      c.addPoint(free_points[0]);
	      c.addPoint(free_points[1]);
	      c.addPoint(free_points[2]);
	      free_points.clear();
	      circles.push_back(c);
	    }
	}
    }

  return num_circles;
}

int RscfCircleModel::getShapeCount(void) const
{
  return circles.size();
}

Circle* RscfCircleModel::getShape(int id) const
{
  if (id < 0 || (unsigned int)id >= circles.size()) {
    return NULL;
  } else {
    return const_cast<Circle*>(circles[id].getCircle()); // or use const Shape* def?!...
  }
}

Circle* RscfCircleModel::getMostLikelyShape(void) const
{
  int cur=0;
  Circle* p = NULL;
  if (circles.size() == 0)
    {
      return NULL;
    }
  else
    {
      printf("Dumping all the circles:\n");
      for (unsigned int i=0; i < circles.size(); ++i)
	{
	  Circle* ptemp = circles[i].getCircle();
	  if (ptemp == NULL || ptemp->radius<MIN_RADIUS || ptemp->radius>MAX_RADIUS)
	    {
	      continue;
	    }
	  printf("\tNo. %d\t(%f, %f)   \tR: %f\tCount: %d\n", i, ptemp->center.x, ptemp->center.y, ptemp->radius, ptemp->count);
	  if (circles[i].getCount() >= circles[cur].getCount())
	    {
	      cur = i;
	      p = ptemp;
	    }
	}
      if(p)
	printf("No. %d is selected\n\n", cur);
      else
	printf("No circle detected.\n\n");
      return const_cast<Circle*>(p); // or use const Shape* definition?!...
    }
}
