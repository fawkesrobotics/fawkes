
/**************************************************************************
 *  lookuptable.h - This header defines a lookup table color model
 *
 *  Created: Fri Jun 10 14:16:52 2005
 *  Copyright  2005  Tim Niemueller  [www.niemueller.de]
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_MODELS_COLOR_LOOKUPTABLE_H_
#define __FIREVISION_MODELS_COLOR_LOOKUPTABLE_H_

#include <fvmodels/color/colormodel.h>

#include <fvutils/colormap/yuvcm.h>
#include <string>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColorModelLookupTable : public ColorModel
{
 public:

  ColorModelLookupTable(YuvColormap *colormap);
  ColorModelLookupTable(const char *file);
  ColorModelLookupTable(const char *file, const char *lut_id, bool destroy_on_free = false);
  ColorModelLookupTable(unsigned int depth, const char *lut_id, bool destroy_on_free);
  ColorModelLookupTable(const char *lut_id, bool destroy_on_free);

  virtual ~ColorModelLookupTable();

  virtual color_t determine(unsigned int y, unsigned int u, unsigned int v) const;

  const char *   get_name();
  YuvColormap *  get_colormap() const;

  void load(const char *filename);

  void set_colormap(const YuvColormap &yuvcm);

  void reset();
  static std::string compose_filename(const std::string format);

  ColorModelLookupTable &  operator+=(const ColorModelLookupTable &cmlt);

 private:
  YuvColormap *__colormap;
};

} // end namespace firevision

#endif
