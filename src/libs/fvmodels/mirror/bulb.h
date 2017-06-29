
/***************************************************************************
 *  bulb.h - class defining a light bulb as mirror
 *
 *  Created: Thu Jul 21 14:25:00 2005
 *  Copyright 2005-2012 Tim Niemueller [www.niemueller.de]
 *            2005      Martin Heracles
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

#ifndef __FIREVISION_MODELS_MIRROR_BULB_H_
#define __FIREVISION_MODELS_MIRROR_BULB_H_

#include <fvmodels/mirror/mirrormodel.h>

#include <string>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SharedMemoryLookupTable;
class BulbGenerator;

class Bulb : public MirrorModel
{
 friend BulbGenerator;

 public:

  // This constructor loads an existing bulb model (lut) from file "filename".
  Bulb(const char *filename);
  Bulb(const char *filename,
       const char *lut_id, bool destroy_on_delete = false);

  Bulb(unsigned int width, unsigned int height);
  Bulb(unsigned int width, unsigned int height,
       const char *lut_id, bool destroy_on_delete = false);

  Bulb(const Bulb &bulb);

  virtual ~Bulb();

  virtual void warp2unwarp(unsigned int warp_x, unsigned int warp_y,
         unsigned int *unwarp_x, unsigned int *unwarp_y);
  virtual void unwarp2warp(unsigned int unwarp_x, unsigned int unwarp_y,
         unsigned int *warp_x, unsigned int *warp_y    );

  virtual const char * getName();

  virtual bool isValid();


  virtual void setWorldPoint(unsigned int image_x,
           unsigned int image_y,
           float        world_r,
           float        world_phi);


  virtual fawkes::polar_coord_2d_t getWorldPointRelative(unsigned int image_x,
               unsigned int image_y  ) const;

  virtual fawkes::cart_coord_2d_t getWorldPointGlobal(unsigned int image_x,
                  unsigned int image_y,
                  float pose_x, float pose_y,
                  float pose_ori ) const;

  virtual void reset();

  virtual fawkes::upoint_t getCenter() const;
  virtual void setCenter(unsigned int image_x,
       unsigned int image_y  );
  virtual void setOrientation(float angle);
  virtual float getOrientation() const;

  virtual bool isValidPoint( unsigned int image_x, unsigned int image_y ) const;


  bool isNonZero(unsigned int image_x,
     unsigned int image_y  ) const;

  unsigned int numNonZero() const;


  float getAngle(unsigned int image_x,
     unsigned int image_y  ) const;

  float getDistanceInImage(unsigned int image_p1_x, unsigned int image_p1_y,
         unsigned int image_p2_x, unsigned int image_p2_y  );

  float convertAngleI2W (float angle_in_image) const;


  void load(const char * filename);
  void save(const char * filename);

  static std::string composeFilename(const char * format);

  const fawkes::polar_coord_2d_t * get_lut() const;

 protected:

  /** bulb file header. */
  typedef struct {
    unsigned int width;   /**< width of LUT */
    unsigned int height;  /**< height of LUT */
    unsigned int center_x;  /**< x coordinate of mirror center in image */
    unsigned int center_y;  /**< y coordinate of mirror center in image */
    float        orientation; /**< orientation of camera in image */
    float        dist_min;  /**< minimum distance from mirror center */
    float        dist_max;  /**< maximum distance from mirror center */
  } bulb_file_header_t;


 private:
  void create();
  void erase();
  void init();

 private:

  // dimension of lut (and image)
  unsigned int width;
  unsigned int height;
  unsigned int bytes_per_sample;

  // center of omni camera device
  unsigned int image_center_x;
  unsigned int image_center_y;

  // orientation of omni camera device
  float orientation;

  // distance of closest and of farthest sample point
  float distance_min;
  float distance_max;

  bool valid;

  char          *lut_id;
  fawkes::polar_coord_2d_t *lut;
  unsigned int   lut_bytes;
  bool destroy_on_delete;

  SharedMemoryLookupTable *shm_lut;


};

} // end namespace firevision

#endif
