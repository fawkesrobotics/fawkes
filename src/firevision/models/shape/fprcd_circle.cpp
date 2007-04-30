
/***************************************************************************
 *  rcd_circle.cpp - Implementation of a circle shape finder
 *
 *  Generated: Sat Sep 10 2005
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

#include <models/shape/accumulators/fit_accum.h>
#include <models/shape/fprcd_circle.h>

#include <cmath>
#include <sys/time.h>

using namespace std;

#define TBY_GRAYSCALE
#ifdef TBY_GRAYSCALE
#define TEST_IF_IS_A_PIXEL(x) ((x)>230)
#else
	#define TEST_IF_IS_A_PIXEL(x) ((x)==0)
#endif // TBY_GRAYSCALE

#define TBY_SQUARED_DIST(x1,y1,x2,y2) \
		(((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2)))
#define TBY_RADIUS_DIFF(x1, y1, x2, y2, r) \
		(((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2))-(r)*(r))


/** @class FprcdCircleModel <models/shape/fprcd_circle.h>
 * FPRCD circle model.
 */

/** Constructor. */
FprcdCircleModel::FprcdCircleModel(void)
{
}

/** Destructor. */
FprcdCircleModel::~FprcdCircleModel(void)
{
	m_Circles.clear();
}

/** Parse image.
 * In this function I implement the circle detection algorithm
 * from the following literature
 *  An Efficient Randomized Algorithm for Detecting 
 * @param buf buffer to parse
 * @param roi ROI to consider
 */
int FprcdCircleModel::parseImage( unsigned char* buf,
			     ROI *roi )
{
  
  unsigned char *buffer = roi->getROIBufferStart(buf);
  unsigned char *line_start = buffer;

  unsigned int     x, y;
  vector<point_t>  pixels,
                   remove_list;
  unsigned int     f = 0;       // number of failures
  int              count;       // number of pixels on the circle
  int              num_circles = 0;
  struct timeval   start, end;

	// clear all the remembered circles
	m_Circles.clear();

	// constants used in the FPRCD algorithm
	const unsigned int	FPRCD_T_f	= 300;	// max. number of failures
	const unsigned int	FPRCD_T_min	= 20;	// min. number of pixels
	const unsigned int	FPRCD_T_d4	= 1;	// dist. threshold for the fourth pixel
	const unsigned int	FPRCD_T_da	= 4;	// dist. threshold for the rest pixels
	const float		FPRCD_T_hw	= 0.7f;	// least number of pixels to be a circle

	const float		FPRCD_RADIUS_MAX= 600.0f;
	const float		FPRCD_RADIUS_MIN= 10.0f;

	// The following constant is not in the stated algorithm
	// I added it in order to find extremely big balls
	// because only a small part of the curve can be
	// observed, so i add an absolute limit.

	// const unsigned int	FPRCD_T_ar	= 250;

	// The following constant is used to
	const int               FPRCD_T_time      = 10000; // = 10 ms (is given in microseconds)

	// The following constant is used for the size of the hollow window in the ROI.
	const float		ROI_HOLLOW_RATE	= 0.00f; // because we use search, no hollow needed.

	const unsigned int roi_hollow_top	= (int)(roi->height * ((1.0f - ROI_HOLLOW_RATE) / 2));
	const unsigned int roi_hollow_bottom	= roi->height - roi_hollow_top;
	const unsigned int roi_hollow_left	= (int)(roi->width * ((1.0f - ROI_HOLLOW_RATE) / 2));
	const unsigned int roi_hollow_right	= roi->width - roi_hollow_left;

	// First, find all the pixels on the edges,
	// and store them in the 'pixels' vector.
	// NEW: excluding the hollow window
	buffer = roi->getROIBufferStart(buf);
	line_start = buffer;

	// Find the boundary of the ball,
	// following used for ball pixel threshold.
	unsigned int boundary_right	= 0;
	unsigned int boundary_bottom	= 0;

	gettimeofday(&start, NULL);
	end.tv_usec = start.tv_usec;

	// top "1/3"
	for (y = 0; y < roi_hollow_top; ++y) {
	  for (x = 0; x < roi->width; ++x) {
	    if (TEST_IF_IS_A_PIXEL(*buffer)) {
	      point_t pt={x, y};
	      pixels.push_back(pt);
	      if (x > boundary_right) boundary_right = x;
	      boundary_bottom = y;
	    }
	    // NOTE: this assumes roi->pixel_step == 1
	    ++buffer;
	  }
	  line_start += roi->line_step;
	  buffer = line_start;
	}
	// middle "1/3"
	for (y = roi_hollow_top; y < roi_hollow_bottom; ++y) {
	  for (x = 0; x < roi_hollow_left; ++x) {
	    if (TEST_IF_IS_A_PIXEL(*buffer)) {
	      point_t pt={x, y};
	      pixels.push_back(pt);
	      if (x > boundary_right) boundary_right = x;
	      boundary_bottom = y;
	    }
 	   // NOTE: this assumes roi->pixel_step == 1
	    ++buffer;
	  }
	  buffer+=(roi_hollow_right - roi_hollow_left);
	  for (x = roi_hollow_right; x < roi->width; ++x) {
	    if (TEST_IF_IS_A_PIXEL(*buffer)) {
	      point_t pt={x, y};
	      pixels.push_back(pt);
	      if (x > boundary_right) boundary_right = x;
	      boundary_bottom = y;
	    }
 	   // NOTE: this assumes roi->pixel_step == 1
	    ++buffer;
	  }
	  line_start += roi->line_step;
	  buffer = line_start;
	}
	// bottom "1/3"
	for (y = roi_hollow_bottom; y < roi->height; ++y) {
	  for (x = 0; x < roi->width; ++x) {
	    if (TEST_IF_IS_A_PIXEL(*buffer)) {
	      point_t pt={x, y};
	      pixels.push_back(pt);
	    }
	    // NOTE: this assumes roi->pixel_step == 1
	    ++buffer;
	  }
	  line_start += roi->line_step;
	  buffer = line_start;
	}
printf("Pixel selected... ");
	// First filter out the lines.
	// Only used for the lines in Koblenz
	const int MAX_LINE_LOOP = 50;
	for (int num_line_loop = 0; num_line_loop < MAX_LINE_LOOP; ++num_line_loop)
	{
		point_t lpt[3];
		for (int i=0; i<3; ++i)
		{
			if (pixels.size()<=1)
			{
				goto out_of_line_filter_loop;
			}
			int id = rand()%pixels.size();
			vector<point_t>::iterator lit = pixels.begin() + id;
			lpt[i] = *lit;
			pixels.erase(lit);
		}
		// Remove the pixels lying on the line
		unsigned int min_x = lpt[0].x;
		unsigned min_x_pos = 0;
		unsigned int max_x = lpt[0].x;
		unsigned max_x_pos = 0;
		unsigned int min_y = lpt[0].y;
		unsigned min_y_pos = 0;
		unsigned int max_y = lpt[0].y;
		unsigned max_y_pos = 0;

		for (unsigned int i=1; i<3; ++i)
		{
			if (lpt[i].x < min_x)
			{
				min_x_pos = i;
				min_x = lpt[i].x;
			}
			if (lpt[i].x > max_x)
			{
				max_x_pos = i;
				max_x = lpt[i].x;
			}
			if (lpt[i].y < min_y)
			{
				min_y_pos = i;
				min_y = lpt[i].y;
			}
			if (lpt[i].y > max_y)
			{
				max_y_pos = i;
				max_y = lpt[i].y;
			}
		}
		int dx, dy, left_point;
		if (max_x - min_x > max_y - min_y)
		{
			dx = max_x - min_x;
			dy = lpt[max_x_pos].y - lpt[min_x_pos].y;
			left_point = 3 - max_x_pos - min_x_pos;
		}
		else
		{
			dx = lpt[max_y_pos].x - lpt[min_y_pos].x;
			dy = max_y - min_y;
			left_point = 3 - max_y_pos - min_y_pos;
		}

		// Check if the 3rd pixel is on the line determined by the first two
		if(fabs(-dy * lpt[left_point].x + dx * lpt[left_point].y + 1) / sqrt(dx*dx+dy*dy) > FPRCD_T_d4)
		{
			// not lying on the line
			for (int i=0; i<3; i++)
				pixels.push_back(lpt[i]);
			continue;
		}

		// check for every pixel in the list whether it is near to the line.
		// if so, just drop it.
		for (unsigned int i=0; i<pixels.size(); ++i)
		{
			if (fabs(-dy * pixels[i].x + dx * pixels[i].y + 1) / sqrt(dx*dx+dy*dy) < FPRCD_T_da)
			{
				vector<point_t>::iterator it = pixels.begin() + i;
				pixels.erase(it);
				--i;
				if (pixels.size() <= 1)
				{
					goto out_of_line_filter_loop;
				}
			}
		}
	}

out_of_line_filter_loop:
	
printf("Line eliminated... ");

	// Then perform the FPRCD algorithm
	point_t pt[4];
	center_in_roi_t center;
	float radius;
	FitAccum fitted_circle;
	vector< point_t >::iterator pos;
	while( (f < FPRCD_T_f) && (pixels.size() > FPRCD_T_min) &&
		( ((end.tv_usec - start.tv_usec) < FPRCD_T_time) ||
		  ((end.tv_usec + 1000000 - start.tv_usec) < FPRCD_T_time) )
		// this only works when time constraint ls small than 500ms of course..
		)
	{
		fitted_circle.reset();
		// Pick four points, and move them to the remove_list.
		for (int i=0; i < 4; ++i)
		{
			int ri = rand() % ((int)pixels.size());
			pos = pixels.begin() + ri;
			pt[i] = *pos; // use * operator of iterator
			pixels.erase(pos);
			remove_list.push_back(pt[i]);
			fitted_circle.addPoint(pt[i]);
		}

		Circle* p = fitted_circle.getCircle();

		if (p == NULL) // There is no circle...
		{
			// Remove the pixels lying on the line
			unsigned int min_x = remove_list[0].x;
			unsigned min_x_pos = 0;
			unsigned int max_x = remove_list[0].x;
			unsigned max_x_pos = 0;
			unsigned int min_y = remove_list[0].y;
			unsigned min_y_pos = 0;
			unsigned int max_y = remove_list[0].y;
			unsigned max_y_pos = 0;

			for (unsigned int i=1; i<remove_list.size(); ++i)
			{
				if (remove_list[i].x < min_x)
				{
					min_x_pos = i;
					min_x = remove_list[i].x;
				}
				if (remove_list[i].x > max_x)
				{
					max_x_pos = i;
					max_x = remove_list[i].x;
				}
				if (remove_list[i].y < min_y)
				{
					min_y_pos = i;
					min_y = remove_list[i].y;
				}
				if (remove_list[i].y > max_y)
				{
					max_y_pos = i;
					max_y = remove_list[i].y;
				}
			}
			int dx, dy;
			if (max_x - min_x > max_y - min_y)
			{
				dx = max_x - min_x;
				dy = remove_list[max_x_pos].y - remove_list[min_x_pos].y;
			}
			else
			{
				dx = remove_list[max_y_pos].x - remove_list[min_y_pos].x;
				dy = max_y - min_y;
			}

			// check for every pixel in the list whether it is near to the line.
			// if so, just drop it.
			for (unsigned int i=0; i<pixels.size(); ++i)
			{
printf("pixel.size() = %ld, i=%d\n", pixels.size(), i);
				if (fabs(-dy * pixels[i].x + dx * pixels[i].y + 1) / sqrt(dx*dx+dy*dy) < FPRCD_T_da)
				{
					vector<point_t>::iterator it = pixels.begin() + i;
					pixels.erase(it);
					--i;
				}
			}

			// Drop all the pixels here.
			remove_list.clear();
			gettimeofday(&end, NULL);
			++f;
			continue;
		}
printf("Algorithm begins... ");
		center = p->center;
		radius = p->radius;

		bool all_near = true;
		for (int i=0; i<4; ++i)
		{
			float dx = center.x - pt[i].x;
			float dy = center.y - pt[i].y;
			if (fabs(sqrt(dx*dx+dy*dy)-radius)>FPRCD_T_d4)
			{
				all_near = false;
				break;
			}
		}
		if (!all_near) // 4 pixels are not lying on a same circle
		{
			++f;
			remove_list.clear();
			gettimeofday(&end, NULL);
printf("Algorithm finished 1.\n");
			continue;
		}

		// count how many pixels are on the circle
		bool changed = true;
		count=0;
		// This count is not done with a single loop,
		// but with multi-loops, until a fixed point is achieved.
		// This is what is new in this algorithm compared with RCD
		while ( changed )
		{
			changed = false;
			count = fitted_circle.getCount();

			Circle* p = fitted_circle.getCircle();
			if (p == NULL) // There is no circle...
			{
				remove_list.clear();
				break;
			}

			center = p->center;
			radius = p->radius;
			unsigned int old_remove_size = remove_list.size();
			for (unsigned int i=0; i < pixels.size(); ++i)
			// to include all the pixels lying near the current fitted circle
			{
				float dx = center.x - pixels[i].x;
				float dy = center.y - pixels[i].y;

				if (fabs(sqrt(dx*dx+dy*dy)-radius) <= FPRCD_T_da)
				{
					changed = true;
					pos = pixels.begin() + i;
					// add this point to the fitted_circle
					fitted_circle.addPoint(*pos);
					// move this pixel to the remove_list
					remove_list.push_back(pixels[i]);
					pixels.erase(pos);
					--i;
				}
			}
			for (unsigned int i=0; i<old_remove_size; ++i)
			// to exclude all the pixels in the current fitted circle list
			// which are actually far away from the circle
			{
				float dx = center.x - remove_list[i].x;
				float dy = center.y - remove_list[i].y;

				if (fabs(sqrt(dx*dx+dy*dy)-radius) > FPRCD_T_da)
				{
					changed = true;
					pos = remove_list.begin() + i;
					// remove this point from the fitted_circle
					fitted_circle.removePoint(*pos);
					// move this pixel to the pixel list
					pixels.push_back(pixels[i]);
					remove_list.erase(pos);
					--i;
					--old_remove_size;
				}
			}
		}
		if (remove_list.size() == 0)
		{
			++f;
			gettimeofday(&end, NULL);
printf("Algorithm finished 2.\n");
			continue;
		}

		if (radius > FPRCD_RADIUS_MAX || radius < FPRCD_RADIUS_MIN)
		// The found circle not in range. Simply drop the pixels.
		{
			remove_list.clear();
			++f;
			gettimeofday(&end, NULL);
printf("Algorithm finished 3.\n");
			continue;
		}

		Circle c;
		c.center = center;
		c.radius = radius;
		c.count  = remove_list.size();
		if (
			c.count >
			( boundary_right > boundary_bottom ?
				boundary_right:
				boundary_bottom
			) * FPRCD_T_hw )
		{
			// This is indeed a circle
			m_Circles.push_back(c);
			++num_circles;
		}

		remove_list.clear();
printf("Algorithm finished 0.\n");
		gettimeofday(&end, NULL);
	}

	return num_circles;
}

int FprcdCircleModel::getShapeCount(void) const
{
	return m_Circles.size();
}

Circle* FprcdCircleModel::getShape(int id) const
{
	if (id < 0 || (unsigned int)id >= m_Circles.size())
	{
		return NULL;
	}
	else
	{
		return const_cast<Circle*>(&m_Circles[id]); // or use const Shape* def?!...
	}
}

Circle* FprcdCircleModel::getMostLikelyShape(void) const
{
	int cur=0;
	switch (m_Circles.size())
	{
	case 0:
		return NULL;
	case 1:
		return const_cast<Circle*>(&m_Circles[0]); // or use const Shape* def?!...
	default:
		for (unsigned int i=1; i < m_Circles.size(); ++i)
			if (m_Circles[i].count > m_Circles[cur].count)
				cur = i;
		return const_cast<Circle*>(&m_Circles[cur]); // or use const Shape* definition?!...
	}
}

void FprcdCircleModel::calcCircle(
		const point_t& p1,
		const point_t& p2,
		const point_t& p3,
		center_in_roi_t& center,
		float& radius)
// Given three points p1, p2, p3,
// this function calculates the center and radius
// of the circle that is determined
{
	const int &x1=p1.x, &y1=p1.y, &x2=p2.x, &y2=p2.y, &x3=p3.x, &y3=p3.y;
	float dx, dy;
	int div = 2*((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1));

	if (div == 0)
	{
		// p1, p2 and p3 are in a straight line.
		radius = -1.0;
		return;
	}
	center.x =	((float)((x2*x2+y2*y2-x1*x1-y1*y1)*(y3-y1)
			-(x3*x3+y3*y3-x1*x1-y1*y1)*(y2-y1))
			/div);
	center.y =	((float)((x2-x1)*(x3*x3+y3*y3-x1*x1-y1*y1)
			-(x3-x1)*(x2*x2+y2*y2-x1*x1-y1*y1))
			/div);
	dx = center.x - x1;
	dy = center.y - y1;
	radius	=	(float)sqrt(dx*dx+dy*dy);
}


