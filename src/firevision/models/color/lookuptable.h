
/**************************************************************************
 *  lookuptable.h - This header defines a lookup table color model
 *
 *  Generated: Fri Jun 10 14:16:52 2005
 *  Copyright  2005  Tim Niemueller  [www.niemueller.de]
 *
 *  $Id$
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_MODELS_COLOR_LOOKUPTABLE_H_
#define __FIREVISION_MODELS_COLOR_LOOKUPTABLE_H_

#include <models/color/colormodel.h>
#include <string>
#include <sys/types.h>

class SharedMemoryLookupTable;

class ColorModelLookupTable : public ColorModel
{
 public:

  ColorModelLookupTable(unsigned int width, unsigned int height, unsigned int depth = 1);
  ColorModelLookupTable(const char *file, unsigned int width, unsigned int height,
			unsigned int depth = 1);

  ColorModelLookupTable(unsigned int width, unsigned int height,
			const char *lut_id, bool destroy_on_free = false);
  ColorModelLookupTable(unsigned int width, unsigned int height, unsigned int depth,
			const char *lut_id, bool destroy_on_free = false);

  ColorModelLookupTable(const char *file, unsigned int width, unsigned int height,
			const char *lut_id, bool destroy_on_free = false);
  ColorModelLookupTable(const char *file, unsigned int width, unsigned int height, 
			unsigned int depth, const char *lut_id, bool destroy_on_free = false);

  virtual ~ColorModelLookupTable();

  color_t       determine(unsigned int y,
			  unsigned int u,
			  unsigned int v) const;

  color_t       determine_cr(unsigned int y,
			     unsigned int u,
			     unsigned int v) const;

  const char *  get_name();
  void          load(const char *file);

  void          save(const char *file);

  void          set(unsigned int y,
		    unsigned int u,
		    unsigned int v,
		    color_t c,
		    bool full_range = true);
  
  void          reset();

  void          set(unsigned char *buffer);

  void          set_range(unsigned int x_max, unsigned int y_max, unsigned int z_max);
  
  unsigned int  size();
  void          to_image(unsigned char *yuv422_planar_buffer);

  unsigned char *  get_buffer();

  ColorModelLookupTable &  operator+=(const ColorModelLookupTable & cmlt);
  ColorModelLookupTable &  operator+=(const char *filename);

  static std::string compose_filename(const std::string format);

 private:

  void create();
  void load_to_buffer(const char *file, unsigned char *buffer, off_t buffer_size);

  char          *lut_id;
  unsigned char *lut;
  unsigned int   lut_bytes;
  bool           destroy_on_free;

  unsigned int width;
  unsigned int height;
  unsigned int depth;
  unsigned int bytes_per_sample;

  SharedMemoryLookupTable *shm_lut;

  unsigned int x_max;
  unsigned int y_max;
  unsigned int z_max;
};




#endif
