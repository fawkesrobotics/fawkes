
/***************************************************************************
 *  bulb.cpp - implements class that defines a light bulb as mirror
 *
 *  Created: Wed Jul 27 16:19:00 2005
 *  Copyright  2005-2007 Tim Niemueller [www.niemueller.de]
 *             2005      Martin Heracles
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

#include <core/exception.h>

#include <fvmodels/mirror/bulb.h>
#include <utils/system/console_colors.h>
#include <fvutils/ipc/shm_lut.h>


#include <cmath>
#include <string>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <sys/utsname.h>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Bulb <fvmodels/mirror/bulb.h>
 * Bulb mirror lookup table.
 * This mirror model is based on a LUT that will map image pixels to radial
 * coordinates in relative radial coordinates.
 * @author Tim Niemueller
 * @author Martin Heracles
 */


/** Constructor.
 * Load bulb LUT from file.
 * @param filename filename of bulb file to load.
 */
Bulb::Bulb(const char *filename)
{
  init();
  load(filename);
}


/** Constructor.
 * Load bulb LUT from file and expose LUT via shared memory.
 * @param filename filename of bulb file to load.
 * @param lut_id LUT ID
 * @param destroy_on_delete destroy LUT on delete
 * @see SharedMemoryLookupTable
 */
Bulb::Bulb(const char *filename,
     const char *lut_id, bool destroy_on_delete)
{
  init();

  this->lut_id            = strdup(lut_id);
  this->destroy_on_delete = destroy_on_delete;

  create();
  load(filename);
}


/** Constructor.
 * Create new empty bulb LUT and expose LUT via shared memory.
 * @param width width of LUT
 * @param height height of LUT
 * @param lut_id LUT ID
 * @param destroy_on_delete destroy LUT on delete
 * @see SharedMemoryLookupTable
 */
Bulb::Bulb(unsigned int width, unsigned int height,
     const char *lut_id, bool destroy_on_delete)
{
  init();

  this->width             = width;
  this->height            = height;
  this->lut_id            = strdup(lut_id);
  this->destroy_on_delete = destroy_on_delete;

  valid = ((width > 0) && (height > 0));

  image_center_x = width / 2;
  image_center_y = height / 2;

  create();
}


/** Constructor.
 * Create new empty bulb LUT.
 * @param width width of LUT
 * @param height height of LUT
 */
Bulb::Bulb(unsigned int width, unsigned int height)
{
  init();

  this->width  = width;
  this->height = height;
  this->lut_id = NULL;

  valid = ((width > 0) && (height > 0));

  image_center_x = width / 2;
  image_center_y = height / 2;

  create();
}


/** Copy constructor.
 * @param bulb bulb LUT to copy
 */
Bulb::Bulb(const Bulb &bulb)
{
  init();

  this->valid = bulb.valid;

  this->width = bulb.width;
  this->height = bulb.height;

  this->image_center_x = bulb.image_center_x;
  this->image_center_y = bulb.image_center_y;

  this->orientation = bulb.orientation;

  this->distance_min = bulb.distance_min;
  this->distance_max = bulb.distance_max;

  create();

  memcpy(lut, bulb.lut, lut_bytes);
}


/** Initializer. */
void
Bulb::init()
{
  valid = false;
  width = 0;
  height = 0;
  lut_id = NULL;
  image_center_x = 0;
  image_center_y = 0;

  // by default, set orientation to 0 rad
  orientation = 0.0;

  // set to the opposite, for later comparison
  distance_min = 999999.0;
  distance_max = 0.0;

  image_center_x = width / 2;
  image_center_y = height / 2;

  shm_lut = 0;
  lut = NULL;
  lut_bytes = 0;

}


/** Destructor.
 * Erases LUT memory. */
Bulb::~Bulb()
{
  erase();
  if ( lut_id != NULL ) {
    free(lut_id);
  }
}


/** Create memory for LUT.
 * This creates the memory segment for the LUT. If a valid LUT ID
 * is set the LUT is exposed via shared memory. Otherwise the LUT is
 * created on the heap.
 */
void
Bulb::create()
{
  bytes_per_sample = sizeof(polar_coord_2d_t);

  if ( lut_id != NULL ) {
    shm_lut   = new SharedMemoryLookupTable( lut_id,
               width, height, /* depth */ 1,
               bytes_per_sample);
    shm_lut->set_destroy_on_delete( destroy_on_delete );
    lut       = (polar_coord_2d_t *)shm_lut->buffer();
    lut_bytes = shm_lut->data_size();
  } else {
    lut_bytes = width * height * bytes_per_sample;
    lut = (polar_coord_2d_t *)malloc( lut_bytes );
  }
  memset(lut, 0, lut_bytes);
}


/** Erase LUT memory. */
void
Bulb::erase()
{
  if ( lut_id != NULL ) {
    delete shm_lut;
    shm_lut = NULL;
    lut = NULL;
    lut_bytes = 0;
  } else {
    if (lut != NULL) {
      free(lut);
    }
    lut = NULL;
    lut_bytes = 0;
  }
}


/** Load LUT from file.
 * @param filename name of LUT file
 */
void
Bulb::load(const char *filename)
{
  FILE *f = fopen(filename, "r");
  if (f == NULL) {
    throw Exception("Cannot open bulb file");
  }

  bulb_file_header_t h;
  if ( (fread(&h, sizeof(h), 1, f) == 0) && (! feof(f)) && (ferror(f) != 0)) {
    throw Exception("Bulb file header invalid");
  }

  width          = h.width;
  height         = h.height;
  image_center_x = h.center_x;
  image_center_y = h.center_y;
  orientation    = h.orientation;
  distance_min   = h.dist_min;
  distance_max   = h.dist_max;

  erase();
  create();

  if ( (fread(lut, lut_bytes, 1, f) == 0) && (! feof(f)) && (ferror(f) != 0)) {
    erase();
    throw Exception("Could not read bulb data from file");
  }

  fclose(f);
}


/** Save LUT from file.
 * @param filename name of LUT file
 */
void
Bulb::save(const char *filename)
{
  if (! valid) {
    throw Exception("Bulb is not valid");
  }

  FILE *f = fopen(filename, "w");

  if (f == NULL) {
    throw Exception("Could not open bulb file for writing");
  }

  bulb_file_header_t h;

  h.width        = width;
  h.height       = height;
  h.center_x     = image_center_x;
  h.center_y     = image_center_y;
  h.orientation  = orientation;
  h.dist_min     = distance_min;
  h.dist_max     = distance_max;

  if ( fwrite(&h, sizeof(h), 1, f) == 0 ) {
    throw Exception("Cannot write bulb file header");
  }


  if ( fwrite(lut, lut_bytes, 1, f) == 0 ) {
    throw Exception("Cannot write bulb file data");
  }

  fclose(f);
}


void
Bulb::warp2unwarp(unsigned int warp_x, unsigned int warp_y,
      unsigned int *unwarp_x, unsigned int *unwarp_y) {
  /*
  // check if image pixel (warp_x, warp_y) maps to something
  if ( this->lut->isNonZero(warp_x, warp_y) ) {
    // get corresponding world point (polar coordinates)
    polar_coord_2d_t worldPoint = this->lut->getWorldPointRelative(warp_x, warp_y);

    // convert to cartesian coordinates
    *unwarp_x = (unsigned int) ( worldPoint.r * cos(worldPoint.phi) );
    *unwarp_y = (unsigned int) ( worldPoint.r * sin(worldPoint.phi) );
  }
  */
}


void
Bulb::unwarp2warp(unsigned int unwarp_x, unsigned int unwarp_y,
      unsigned int *warp_x, unsigned int *warp_y    )
{

}


const char *
Bulb::getName() {
  return "Mirrormodel::Bulb";
}


/** Check if a valid LUT has been loaded.
 * @return true if a valid LUT has been loaded and can be used, false otherwise
 */
bool
Bulb::isValid()
{
  return valid;
}


polar_coord_2d_t
Bulb::getWorldPointRelative(unsigned int image_x,
          unsigned int image_y) const
{
  if ( (image_x > width) || (image_y > height) ) {
    polar_coord_2d_t rv;
    rv.r = rv.phi = 0;
    return rv;
  } else {
    // will be tuned
    polar_coord_2d_t rv;
    rv.r   = lut[image_y * width + image_x].r;
    rv.phi = lut[image_y * width + image_x].phi;
    return rv;

  }
}


cart_coord_2d_t
Bulb::getWorldPointGlobal(unsigned int image_x,
        unsigned int image_y,
        float pose_x,
        float pose_y,
        float pose_ori        ) const
{

  cart_coord_2d_t rv;
  rv.x = 0;
  rv.y = 0;

  if (image_x > width) return rv;
  if (image_y > height) return rv;


  // get relative world point (polar coordinates)
  polar_coord_2d_t pointRelative;
  pointRelative = getWorldPointRelative( image_x, image_y );

  // convert relative angle "pointRelative.phi" to global angle "globalPhi"
  // (depends on "robOri")
  float globalPhi;
  if ( pose_ori                   >= 0.0  &&
       pointRelative.phi          >= 0.0  &&
       pointRelative.phi + pose_ori >  M_PI    ) {
    globalPhi = -( 2*M_PI - (pointRelative.phi + pose_ori) );
  } else if ( pose_ori                     < 0.0   &&
        pointRelative.phi          < 0.0   &&
        pointRelative.phi + pose_ori < -M_PI    ) {
    globalPhi = 2*M_PI - fabs( pointRelative.phi + pose_ori );
  } else {
    globalPhi = pointRelative.phi + pose_ori;
  }

  // convert relative world point to global world point
  // (using global angle "globalPhi" instead of relative angle "pointRelative.phi")
  rv.x = pointRelative.r * cos( globalPhi ) + pose_x;
  rv.y = pointRelative.r * sin( globalPhi ) + pose_y;

  return rv;
}


/** Get the raw lookup table.
 * Returns a pointer to the raw lookup table buffer ordered in row-major
 * mappings from pixels to polar coordinates.
 * @return raw lookup table
 */
const fawkes::polar_coord_2d_t *
Bulb::get_lut() const
{
  return lut;
}


/** Set a world point mapping.
 * This modifies the mapping in the LUT. An exception is thrown if the coordinates
 * are out of range or the distance is zero.
 * @param image_x x coordinate of point in image in pixels
 * @param image_y y coordinate of point in image in pixels
 * @param world_r distance to real object from camera center in meters
 * @param world_phi angle to real object
 */
void
Bulb::setWorldPoint(unsigned int image_x,
        unsigned int image_y,
        float        world_r,
        float        world_phi)
{
  if (image_x > width) {
    throw Exception("MirrorModel::Bulb::setWorldPoint(): image_x out of bounds");
  }
  if (image_y > height) {
    throw Exception("MirrorModel::Bulb::setWorldPoint(): image_x out of bounds");
  }
  if (world_r == 0.f) {
    throw Exception("MirrorModel::Bulb::setWorldPoint(): radius cannot be zero");
  }

  // set world point
  lut[image_y * width + image_x].r   = world_r;
  lut[image_y * width + image_x].phi = world_phi; //convertAngleI2W( world_phi );

  // update distances
  float dist_new = getDistanceInImage( image_x, image_y,
               image_center_x, image_center_y );
  if (dist_new > distance_max) {
    distance_max = dist_new;
  }
  if (dist_new < distance_min) {
    distance_min = dist_new;
  }
}


void
Bulb::reset()
{
  memset(lut, 0, lut_bytes);
}


upoint_t
Bulb::getCenter() const
{
  upoint_t center;

  center.x = image_center_x;
  center.y = image_center_y;

  return center;
}


void
Bulb::setCenter(unsigned int image_x,
    unsigned int image_y  )
{
  if (image_x > width) {
    throw Exception("MirrorModel::Bulb::setCenter(): image_x out of bounds");
  }
  if (image_y > height) {
    throw Exception("MirrorModel::Bulb::setCenter(): image_y out of bounds");
  }

  image_center_x = image_x;
  image_center_y = image_y;

  // caution: the distance_min and distance_max values are not correct afterwards!
}


void
Bulb::setOrientation(float angle)
{
  if (angle >= -M_PI &&
      angle <=  M_PI    ) {
    // angle is valid
    orientation = angle;
  } else {
    // angle not valid
    throw Exception("MirrorModel::Bulb::setOrientation(): angle is invalid");
  }
}


float
Bulb::getOrientation() const
{
  return orientation;
}


bool
Bulb::isValidPoint(unsigned int image_x, unsigned int image_y) const
{
  return isNonZero(image_x, image_y);
}


/** Check if pixel maps to valid world point.
 * @param image_x x coordinate in image
 * @param image_y y coordinate in image
 * @return true, iff image pixel (imagePointX, imagePointY) is not zero
 * (checks distances "r" only, not the angles "phi") i.e. if it maps to a
 * real-world position
 */
bool
Bulb::isNonZero(unsigned int image_x,
    unsigned int image_y  ) const
{
  if (image_x > width) return false;
  if (image_y > height) return false;

  return (lut[image_y * width + image_x].r != 0.0);
}


/** Get number of non-zero entries.
 * @return number of non-zero entries.
 */
unsigned int
Bulb::numNonZero() const
{
  unsigned int num_nonzero = 0;
  for (unsigned int h = 0; h < height; ++h) {
    for (unsigned int w = 0; w < width; ++w) {
      if ( lut[h * width + w].r != 0.0 ) {
  ++num_nonzero;
      }
    }
  }

  return num_nonzero;
}

/** Angle between direction to point and "to the right".
 * @param image_x x coordinate in image
 * @param image_y y coordinate in image
 * @return angle between direction "to point (px, py)" and direction "to the right",
 * with respect to center point. (Angle is in radians; clockwise is positive,
 * counter-clockwise is negative.)
 */
float
Bulb::getAngle(unsigned int image_x,
         unsigned int image_y  ) const
{
  return atan2f((float(image_y) - float(image_center_y)),
    (float(image_x) - float(image_center_x)));
}


/** Euklidean distance between to image points.
 * @return the (euklidian) distance between two image points
 * @param image_p1_x x coordinate in image of point 1
 * @param image_p1_y y coordinate in image of point 1
 * @param image_p2_x x coordinate in image of point 2
 * @param image_p2_y y coordinate in image of point 2
 */
float
Bulb::getDistanceInImage(unsigned int image_p1_x, unsigned int image_p1_y,
       unsigned int image_p2_x, unsigned int image_p2_y  )
{
  float diffX = float(image_p1_x) - float(image_p2_x);
  float diffY = float(image_p1_y) - float(image_p2_y);

  return sqrt( diffX * diffX +
         diffY * diffY   );
}


/** convertAngleI2W
 * @return If you have a (ball-) direction in the omni-image,
 * at which direction is the ball in the world,
 * relative to the robot?
 * @param angle_in_image angle to be converted
 */
float
Bulb::convertAngleI2W (float angle_in_image) const
{
  // get rid of impact of "orientation" on angle_in_image
  if (angle_in_image - orientation >= -M_PI &&
      angle_in_image - orientation <=  M_PI   ) {
    angle_in_image = angle_in_image - orientation;
  }
  else if (angle_in_image - orientation > M_PI) {
    angle_in_image = -( M_PI - ((angle_in_image - orientation) - M_PI) );
  }
  else { // "angle_in_image - orientation < -M_PI"
    angle_in_image = M_PI - ( (-(angle_in_image - orientation)) - M_PI );
  }

  // turn around by PI
  // (this is taking the angle that points to the opposite direction)
  if (angle_in_image + M_PI >= -M_PI &&
      angle_in_image + M_PI <= M_PI    ) {
    angle_in_image = angle_in_image + M_PI;
  }
  else if (angle_in_image + M_PI > M_PI) {
    angle_in_image = -( M_PI - angle_in_image );
  }
  else { // "angle_in_image + M_PI < -M_PI"
    angle_in_image = M_PI - ( (-(angle_in_image + M_PI)) - M_PI );
  }

  // convert without taking into consideration "orientation"
  // (flipping at vertical axis)
  if (angle_in_image >= 0.0 &&
      angle_in_image <= M_PI  ) {
    angle_in_image = (-angle_in_image + M_PI);
  } else if (angle_in_image >= -M_PI &&
       angle_in_image <= 0.0     ) {
    angle_in_image = (-angle_in_image - M_PI);
  } else if (angle_in_image > M_PI) {
    // Clip
    angle_in_image = M_PI;
  } else if (angle_in_image < -M_PI) {
    // Clip
    angle_in_image = -M_PI;
  } else {      // should not occurr
    cout << "Bulb::convertAngleI2W: ERROR! An invalid angle occurred (angle="
   << angle_in_image << ")." << endl;
    return 0.0;
  }

  return angle_in_image;
}


/** Compose a filename matching the given format.
 * In the format %h is replaced by the hostname.
 * @param format format of file name
 * @return filename based on the given format
 */
string
Bulb::composeFilename(const char *format)
{
  string rv = format;

  struct utsname uname_info;
  uname( &uname_info );

  size_t loc = rv.find( "%h" );
  if (loc != string::npos) {
    rv.replace( loc, 2, uname_info.nodename );
  }

  return rv;
}

} // end namespace firevision
