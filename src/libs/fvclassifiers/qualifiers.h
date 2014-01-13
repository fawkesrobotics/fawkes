/***************************************************************************
 *  qualifiers.h - Pixel qualifier
 *
 *  Created: Mon, 09. Jun 2008 22:54
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
 *
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


#ifndef __FIREVISION_APPS_NAO_LOC_QUALIFIERS_H_
#define __FIREVISION_APPS_NAO_LOC_QUALIFIERS_H_

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Qualifier
{
 public:
  Qualifier();
  virtual ~Qualifier();

  /** Getter.
   * @param pixel the pixel of interest
   * @return a corresponding int value
   */
  virtual int   get(fawkes::upoint_t pixel) = 0;

  virtual unsigned char* get_buffer();
  virtual void set_buffer(unsigned char* buffer, unsigned int width = 0,
        unsigned int height = 0);

  virtual colorspace_t get_colorspace();
  virtual void set_colorspace(colorspace_t colorspace);


 protected:
  Qualifier(unsigned char* buffer, unsigned int width,
      unsigned int height, colorspace_t colorspace);

  /** Image buffer */
  unsigned char* buffer_;

  /** Width of the buffer */
  unsigned int width_;
  /** Height of the buffer */
  unsigned int height_;

  /** Size of the buffer */
  unsigned int size_;

  /** Colorspace of the buffer */
  colorspace_t colorspace_;
};


class LumaQualifier: public Qualifier
{
 public:
  LumaQualifier() {};
  LumaQualifier(unsigned char* buffer, unsigned int width,
    unsigned int height, colorspace_t colorspace);
  virtual ~LumaQualifier() {};

  virtual int   get(fawkes::upoint_t pixel);
};


class SkyblueQualifier: public Qualifier
{
 public:
  SkyblueQualifier() {};
  SkyblueQualifier(unsigned char* buffer, unsigned int width,
       unsigned int height, colorspace_t colorspace);
  virtual ~SkyblueQualifier() {};

  virtual int   get(fawkes::upoint_t pixel);


 private:
  static const unsigned int threshold_ = 128;
};


class YellowQualifier: public Qualifier
{
 public:
  YellowQualifier() {};
  YellowQualifier(unsigned char* buffer, unsigned int width,
      unsigned int height, colorspace_t colorspace);
  virtual ~YellowQualifier() {};

  virtual int   get(fawkes::upoint_t pixel);


 private:
  static const unsigned int threshold_ = 100;
};

} // end namespace firevision

#endif // __FIREVISION_APPS_NAO_LOC_QUALIFIERS_H_
