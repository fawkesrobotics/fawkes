
/***************************************************************************
 *  bulb_generator.cpp - generator for bulb lookup tables
 *
 *  Generated: Thu Mar 23 20:40:27 2006
 *  Copyright  2005-2006 Tim Niemueller [www.niemueller.de]
 *             2005      Martin Heracles
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

#include <models/mirror/bulb/bulb_generator.h>
#include <models/mirror/bulb/bulb_sampler.h>
#include <models/mirror/bulb.h>

#include <utils/math/angle.h>

#include <cmath>
#include <list>
#include <utility>

using namespace std;

/** @class BulbGenerator <models/mirror/bulb/bulb_generator.h>
 * Bulb Lookuptable Generator.
 * This generator processes samples taken with BulbSampler and calculates the
 * mirror lookup table.
 * @author Martin Herakles
 * @author Tim Niemueller
 */

/** Constructor.
 * @param sampler bulb sampler
 * @param handler progress handler, will informed about progress
 */
BulbGenerator::BulbGenerator(BulbSampler *sampler,
			     BulbGeneratorProgressHandler *handler)
{
  this->sampler = sampler;
  this->handler = handler;

  data   = sampler->getBulb();
  result = new Bulb(*data);

  width    = data->width;
  height   = data->height;
  center_x = data->image_center_x;
  center_y = data->image_center_y;

  data_lut = data->lut;
  res_lut = result->lut;
}


/** Destructor. */
BulbGenerator::~BulbGenerator()
{
  delete result;
}


/** Generate LUT. */
void
BulbGenerator::generate()
{

  unsigned int totalNrOfPixels = width * height;
  unsigned int nrOfProcessedPixels = 0;
  unsigned int num_non_zero_values = 0;

  handler->setTotalSteps(totalNrOfPixels);

  /* initialization of sector points
     (For each of the four "Quadranten" of the "local coordinate system" with origin (r, phi),
     the sample point closest to point (r, phi) is called a sector point of (r, phi). ) */
  float r_1, phi_1;
  float r_2, phi_2;
  float r_3, phi_3;
  float r_4, phi_4;
  bool sector_valid = false;
  r_1 = r_2 = r_3 = r_4 = 0.0;
  phi_1 = phi_2 = phi_3 = phi_4 = 0.0;

  // sector points (cartesian coordinates)
  unsigned int u_1 = 0;
  unsigned int v_1 = 0;
  float dist_1_minimal = 100000.0;
  unsigned int u_2 = 0;
  unsigned int v_2 = 0;
  float dist_2_minimal = 100000.0;
  unsigned int u_3 = 0;
  unsigned int v_3 = 0;
  float dist_3_minimal = 100000.0;
  unsigned int u_4 = 0;
  unsigned int v_4 = 0;
  float dist_4_minimal = 100000.0;

  list< pair< cart_coord_t, polar_coord_t > > data_points;
  list< pair< cart_coord_t, polar_coord_t > >::iterator dpi;

  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if ( data->isNonZero(x, y) ) {
	cart_coord_t  c;
	c.x = x;
	c.y = y;
	polar_coord_t p;
	p.r   = data->getDistanceInImage( x, y, center_x, center_y );
	p.phi = data->getAngle(x, y);
	data_points.push_back( pair<cart_coord_t, polar_coord_t>(c, p) );
      }
    }
  }


  // interpolation
  float dist = 0.0;
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      dist = data->getDistanceInImage( x, y, center_x, center_y );
      if ( ! data->isNonZero(x, y) &&
	   dist <= data->distance_max    &&
	   dist >= data->distance_min       ) {	
	/* non-sample point, 
	   and within the (distance_min, distance_max) interval
	   ---> has to be interpolated */

	// convert (x, y) to polar coordinates (r, phi)
	float r = sqrt( (float(x) - center_x) *
			(float(x) - center_x)   +
			(float(y) - center_y) *
			(float(y) - center_y)     );
	float phi =  data->getAngle(x, y);

	// if point is outside of old sector...
	if ( ! (r >= r_1 &&
		r <= r_2 &&
		phi >= phi_1 &&
		phi >= phi_2 &&
		r >= r_3 &&
		r <= r_4 &&
		phi <= phi_3 &&
		phi <= phi_4   ) ) {
	  // ...find the correct sector

	  // reset sector point data
	  // (but keep the r and phi sector point data)
	  sector_valid = false;
	  u_1 = 0;
	  v_1 = 0;
	  dist_1_minimal = 100000.0;
	  u_2 = 0;
	  v_2 = 0;
	  dist_2_minimal = 100000.0;
	  u_3 = 0;
	  v_3 = 0;
	  dist_3_minimal = 100000.0;
	  u_4 = 0;
	  v_4 = 0;
	  dist_4_minimal = 100000.0;
	  
	  for (dpi = data_points.begin(); ! sector_valid && (dpi != data_points.end()); ++dpi) {
	    unsigned int u = (*dpi).first.x;
	    unsigned int v = (*dpi).first.y;
		
	    // convert to polar coordinates (r_sample, phi_sample)
	    float r_sample = sqrt( (float(u) - center_x) *
				   (float(u) - center_x)   +
				   (float(v) - center_y) *
				   (float(v) - center_y)     );
	    float phi_sample = data->getAngle(u, v);
		
	    // calculate distance between (r_sample, phi_sample) and (r, phi)
	    float dist = sqrt( (fabs(float(x)) - fabs(float(u))) *
			       (fabs(float(x)) - fabs(float(u)))   +
			       (fabs(float(y)) - fabs(float(v))) *
			       (fabs(float(y)) - fabs(float(v)))     );

	    // update sector points
	    if (dist < dist_1_minimal && 
		phi_sample <= phi &&
		r_sample <= r) {
	      dist_1_minimal = dist;
	      u_1 = u;
	      v_1 = v;
	      //cout << "found 1 at (" << u << "," << v << ")" << endl;
	    } else if (dist < dist_2_minimal && 
		       phi_sample <= phi &&
		       r_sample > r) {
	      dist_2_minimal = dist;
	      u_2 = u;
	      v_2 = v;
	      //cout << "found 2 at (" << u << "," << v << ")" << endl;
	    } else if (dist < dist_3_minimal && 
		       phi_sample > phi &&
		       r_sample <= r) {
	      dist_3_minimal = dist;
	      u_3 = u;
	      v_3 = v;
	      //cout << "found 3 at (" << u << "," << v << ")" << endl;
	    } else if (dist < dist_4_minimal && 
		       phi_sample > phi &&
		       r_sample > r) {
	      dist_4_minimal = dist;
	      u_4 = u;
	      v_4 = v;
	      //cout << "found 4 at (" << u << "," << v << ")" << endl;
	    }
	  }

	  if (dist_1_minimal < 100000.0 &&
	      dist_2_minimal < 100000.0 &&
	      dist_3_minimal < 100000.0 &&
	      dist_4_minimal < 100000.0) {	
	    // all four sector points have been found
	    
	    // convert to polar coordinates
	    r_1 = sqrt( (float(u_1) - center_x) *
			(float(u_1) - center_x)   +
			(float(v_1) - center_y) *
			(float(v_1) - center_y)     );
	    phi_1 = data->getAngle(u_1, v_1);
	    r_2   = sqrt( (float(u_2) - center_x) *
			  (float(u_2) - center_x)   +
			  (float(v_2) - center_y) *
			  (float(v_2) - center_y)     );
	    phi_2 = data->getAngle(u_2, v_2);
	    r_3   = sqrt( (float(u_3) - center_x) *
			  (float(u_3) - center_x)   +
			  (float(v_3) - center_y) *
			  (float(v_3) - center_y)     );
	    phi_3 = data->getAngle(u_3, v_3);
	    r_4   = sqrt( (float(u_4) - center_x) *
			  (float(u_4) - center_x)   +
			  (float(v_4) - center_y) *
			  (float(v_4) - center_y)     );
	    phi_4 = data->getAngle(u_4, v_4);
	    
	    sector_valid = true;
	  }

	} // end if point is outside of old sector
	

	if (! sector_valid &&
	    ( (phi < -M_PI/2) ||
	      (phi > M_PI/2) )
	    ) {
	  // sector could not be found, give it another try in the
	  // interval (-M_PI/2, M_PI/2)
	  // EvilHack(TiM).
	  // This is needed to work around Martin's bullshit of blind region

	  for (dpi = data_points.begin(); ! sector_valid && (dpi != data_points.end()); ++dpi) {
	    unsigned int u = (*dpi).first.x;
	    unsigned int v = (*dpi).first.y;
		
	    // convert to polar coordinates (r_sample, phi_sample)
	    float phi_sample = normalize_rad( data->getAngle(u, v) );

	    if ( (phi_sample >= (M_PI * 3/2)) ||
		 (phi_sample <= M_PI/2) ) {
	      // not in interesting region
	      continue;
	    }

	    float normal_phi = normalize_rad( phi );
	    float r_sample = sqrt( (float(u) - center_x) *
				   (float(u) - center_x)   +
				   (float(v) - center_y) *
				   (float(v) - center_y)     );

	    // calculate distance between (r_sample, phi_sample) and (r, phi)
	    float dist = sqrt( (fabs(float(x)) - fabs(float(u))) *
			       (fabs(float(x)) - fabs(float(u)))   +
			       (fabs(float(y)) - fabs(float(v))) *
			       (fabs(float(y)) - fabs(float(v)))     );


	    // update sector points
	    if (dist < dist_1_minimal && 
		phi_sample <= normal_phi &&
		r_sample <= r) {
	      dist_1_minimal = dist;
	      u_1 = u;
	      v_1 = v;
	      //cout << "found 1 at (" << u << "," << v << ")" << endl;
	    } else if (dist < dist_2_minimal && 
		       phi_sample <= normal_phi &&
		       r_sample > r) {
	      dist_2_minimal = dist;
	      u_2 = u;
	      v_2 = v;
	      //cout << "found 2 at (" << u << "," << v << ")" << endl;
	    } else if (dist < dist_3_minimal && 
		       phi_sample > normal_phi &&
		       r_sample <= r) {
	      dist_3_minimal = dist;
	      u_3 = u;
	      v_3 = v;
	      //cout << "found 3 at (" << u << "," << v << ")" << endl;
	    } else if (dist < dist_4_minimal && 
		       phi_sample > normal_phi &&
		       r_sample > r) {
	      dist_4_minimal = dist;
	      u_4 = u;
	      v_4 = v;
	      //cout << "found 4 at (" << u << "," << v << ")" << endl;
	    }
	  }

	  if (dist_1_minimal < 100000.0 &&
	      dist_2_minimal < 100000.0 &&
	      dist_3_minimal < 100000.0 &&
	      dist_4_minimal < 100000.0) {	
	    // all four sector points have been found
	    
	    // convert to polar coordinates
	    r_1 = sqrt( (float(u_1) - center_x) *
			(float(u_1) - center_x)   +
			(float(v_1) - center_y) *
			(float(v_1) - center_y)     );
	    phi_1 = data->getAngle(u_1, v_1);
	    r_2   = sqrt( (float(u_2) - center_x) *
			  (float(u_2) - center_x)   +
			  (float(v_2) - center_y) *
			  (float(v_2) - center_y)     );
	    phi_2 = data->getAngle(u_2, v_2);
	    r_3   = sqrt( (float(u_3) - center_x) *
			  (float(u_3) - center_x)   +
			  (float(v_3) - center_y) *
			  (float(v_3) - center_y)     );
	    phi_3 = data->getAngle(u_3, v_3);
	    r_4   = sqrt( (float(u_4) - center_x) *
			  (float(u_4) - center_x)   +
			  (float(v_4) - center_y) *
			  (float(v_4) - center_y)     );
	    phi_4 = data->getAngle(u_4, v_4);
	    
	    sector_valid = true;
	  }

	}

	if (sector_valid) {
	  // interpolate radius:
	  float r_1_2_interpolated = data_lut[v_1 * width + u_1].r +
	    ( (data_lut[v_2 * width + u_2].r - data_lut[v_1 * width + u_1].r) / (r_2 - r_1) )
	    * (r - r_1);
	  float r_3_4_interpolated = data_lut[v_3 * width + u_3].r +
	    ( (data_lut[v_4 * width + u_4].r - data_lut[v_3 * width + u_3].r) / (r_4 - r_3) )
	    * (r - r_3);

	  // calculate average angle of first and second sector point
	  // float phi_avg_1_2 = (phi_1 + phi_2) / 2.0;
	  // calculate average angle of third and fourth sector point
	  // float phi_avg_3_4 = (phi_3 + phi_4) / 2.0;

	  float r_interpolated = (r_1_2_interpolated + r_3_4_interpolated) / 2.0;
	  /*
	    ( (r_3_4_interpolated - 
	       r_1_2_interpolated   ) / 
	      (phi_avg_3_4 - phi_avg_1_2) ) *
	    (phi - phi_avg_1_2);
	  */

	  // for simplicity, do not really interpolate angle;
	  // assume that bulb is "rotationssymmetrisch"
	  float phi_interpolated = data->getAngle( x, y );

	  /*
	  // interpolate angle:
	  float phi_1_3_interpolated = 
	    this->lut[u_1][v_1].phi +
	    ( (this->lut[u_3][v_3].phi -
	       this->lut[u_1][v_1].phi   ) / 
	      (phi_3 - phi_1)                ) *
	    (phi - phi_1);

	  float phi_2_4_interpolated = 
	    this->lut[u_2][v_2].phi +
	    ( (this->lut[u_4][v_4].phi -
	       this->lut[u_2][v_2].phi   ) / 
	      (phi_4 - phi_2)                ) *
	    (phi - phi_2);

	  // calculate average radius of first and third sector point
	  float r_avg_1_3 = (r_1 + r_3) / 2.0;
	  // calculate average radius of second and fourth sector point
	  float r_avg_2_4 = (r_2 + r_4) / 2.0;

	  float phi_interpolated = 
	    phi_1_3_interpolated +
	    ( (phi_2_4_interpolated - 
	       phi_1_3_interpolated   ) / 
	      (r_avg_2_4 -
	       r_avg_1_3   )              ) *
	    (r - r_avg_1_3);
	  */

	  // write interpolated values to lutCopy at (x, y)
	  res_lut[y * width + x].r   = r_interpolated;
	  res_lut[y * width + x].phi = phi_interpolated;

	  //cout << "setting res_lut[" << y * width + x << "/" << x << "," << y
	  //     << "] = (" << r_interpolated << "," << phi_interpolated << ")" << endl;

	  ++num_non_zero_values;
	  
	} else { // end if sector valid
	  // cout << "sector invalid" << endl;
	  /*
	  if ( phi > M_PI-0.1 ||
	       phi < -M_PI+0.1 ) {
	    cout << "is: r=" << r << "  phi=" << phi << endl;
	    cout << "u_1=" << u_1 << "  v_1=" << v_1 << "  dist_1_minimal=" << dist_1_minimal << endl;
	    cout << "u_2=" << u_2 << "  v_2=" << v_2 << "  dist_2_minimal=" << dist_2_minimal << endl;
	    cout << "u_3=" << u_3 << "  v_3=" << v_3 << "  dist_3_minimal=" << dist_3_minimal << endl;
	    cout << "u_4=" << u_4 << "  v_4=" << v_4 << "  dist_4_minimal=" << dist_4_minimal << endl;
	  }
	  */
	}
	
      } else { // end if to be interpolated
	//cout << "no need to interpolate pixel (" << x << "," << y << ")" << endl;
      }
      
      // whether interpolated or not, count this pixel as processed
      ++nrOfProcessedPixels;
      
    } // end inner for
    
    handler->setProgress(nrOfProcessedPixels);

  } // end outer for 

  handler->finished();

  // cout << "Number of non-zero values: " << num_non_zero_values << endl;

}


/** Get the result.
 * @return Bulb mirror model.
 */
Bulb *
BulbGenerator::getResult()
{
  return result;
}
