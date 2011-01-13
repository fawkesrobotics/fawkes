
/***************************************************************************
 *  pnm.cpp - Implementation of a PNM writer
 *
 *  Generated: Mon Feb 06 19:18:03 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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
#include <core/exceptions/system.h>
#include <fvutils/writers/pnm.h>
#include <fvutils/color/conversions.h>

#include <cstdio>
#include <cstdlib>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PNMWriter <fvutils/writers/pnm.h>
 * PNM file writer.
 */

/** Constructor.
 * @param format PNM subformat
 */
PNMWriter::PNMWriter(PNMFormat format)
  : Writer("pnm")
{
  this->format = format;

  buffer_size = calc_buffer_size();
  buffer = (unsigned char *)malloc(buffer_size);
  buffer_start = buffer;
}

/** Constructor.
 * @param format PNM subformat
 * @param filename filename
 * @param width image width
 * @param height image height
 */
PNMWriter::PNMWriter(PNMFormat format, const char *filename, unsigned int width, unsigned int height)
  : Writer("pnm")
{
  set_filename(filename);

  this->format = format;
  this->width = width;
  this->height = height;

  buffer_size = calc_buffer_size();
  buffer = (unsigned char *)malloc(buffer_size);
  buffer_start = buffer;
}


void
PNMWriter::set_buffer(colorspace_t cspace, unsigned char *yuv422_planar_buf)
{
  if (cspace != YUV422_PLANAR) {
    throw Exception("Unsupported colorspace, PNM can only write YUV422_PLANAR images");
  }

  buffer = buffer_start;
  memset(buffer, 0, buffer_size);

  buffer += write_header();

  unsigned char *yp, *up, *vp;
  unsigned char y1, y2, u, v;

  yp = yuv422_planar_buf;
  up = YUV422_PLANAR_U_PLANE(yuv422_planar_buf, width, height);
  vp = YUV422_PLANAR_V_PLANE(yuv422_planar_buf, width, height);


  if ( (format == PNM_PBM) ||
       (format == PNM_PBM_ASCII) ) {

    unsigned char byte     = 0;
    unsigned int  num_bits = 0;

    for (unsigned int i = 0; i < height; ++i) {
      byte = 0;
      num_bits = 0;
      for (unsigned int j = 0; j < width; ++j) {
	y1 = *yp++;
	if (y1 > 127) {
	  y2 = 1;
	} else {
	  y2 = 0;
	}
	if ( format == PNM_PBM ) {
	  byte |= (y2 << (7-num_bits++));
	  if (num_bits == 8) {
	    *buffer++ = byte;
	    byte = 0;
	    num_bits = 0;
	  }
	} else {
	  // PNM_PBM_ASCII
	  sprintf((char *)buffer, "%c ", y2);
	  buffer += 2;
	}
      }
      if ((format == PNM_PBM) && (num_bits != 0)) {
	*buffer++ = byte;
      }
    }
  } else if ( (format == PNM_PGM) ||
	      (format == PNM_PGM_ASCII) ) {

    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
	y1 = *yp++;
	if ( format == PNM_PGM ) {
	  *buffer++ = y1;
	} else {
	  // PNM_PGM_ASCII
	  sprintf((char *)buffer, "%3c ", y1);
	  buffer += 4;
	}
      }
    }

  } else if ( format == PNM_PPM ) {

    convert(YUV422_PLANAR, RGB, yuv422_planar_buf, buffer, width, height);

  } else if (format == PNM_PPM_ASCII) {

    unsigned char r, g, b;

    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < (width / 2); ++j) {
	y1 = *yp++;
	y2 = *yp++;
	u  = *up++;
	v  = *vp++;
	
	pixel_yuv_to_rgb(y1, u, v, &r, &g, &b);
	sprintf((char *)buffer, "%3c %3c %3c  ", r, g, b);
	buffer += 13;

	pixel_yuv_to_rgb(y2, u, v, &r, &g, &b);
	sprintf((char *)buffer, "%3c %3c %3c  ", r, g, b);
	buffer += 13;
      }
    }
  }

}


const char *
PNMWriter::format2string(PNMFormat format)
{
  switch ( format ) {
  case PNM_PBM:        return "P4";
  case PNM_PBM_ASCII:  return "P1";
  case PNM_PGM:        return "P5";
  case PNM_PGM_ASCII:  return "P2";
  case PNM_PPM:        return "P6";
  case PNM_PPM_ASCII:  return "P3";

  default:
    throw Exception("Unknown PNMFormat");
  }
}

unsigned int
PNMWriter::write_header(bool simulate)
{
  unsigned int rv = 25;

  if (! simulate) {
    switch ( format ) {
    case PNM_PBM:
    case PNM_PBM_ASCII:
      sprintf((char *)buffer, "%s %10u %10u\n", format2string(format), width, height);
      break;

    case PNM_PGM:
    case PNM_PGM_ASCII:
    case PNM_PPM:
    case PNM_PPM_ASCII:
      sprintf((char *)buffer, "%s %10u %10u 255\n", format2string(format), width, height);
      break;
    default: break;
    }
  }

  switch (format) {
  case PNM_PGM:
  case PNM_PGM_ASCII:
  case PNM_PPM:
  case PNM_PPM_ASCII:
    rv += 4;
    break;
  default: break;
  }

  return rv;
}


void
PNMWriter::write()
{
  FILE *fp = fopen(filename, "wb");
  if (!fp) {
    throw Exception("Could not open file for writing");
  }

  if (fwrite(buffer_start, buffer_size, 1, fp) != 1) {
    throw FileWriteException(filename, "Failed to write data");
  }
  fclose(fp);

}


unsigned int
PNMWriter::calc_buffer_size()
{
  unsigned int rv = write_header(true);

  unsigned int num_row_bytes = 0;

  switch ( format ) {
  case PNM_PBM:
    // full bytes
    num_row_bytes = width / 8;
    if ((width % 8) != 0) {
      // possibly the last non-full byte
      num_row_bytes += 1;
    }
    break;

  case PNM_PBM_ASCII:
    // width numbers + width - 1 white spaces + \n
    num_row_bytes = 2 * width;
    break;

  case PNM_PGM:
    num_row_bytes = width;
    break;

  case PNM_PGM_ASCII:
    num_row_bytes = width * 4;
    break;

  case PNM_PPM:
    num_row_bytes = 3 * width;
    break;

  case PNM_PPM_ASCII:
    // why 13?
    // 3 + 1  for each number (0 to 255) per component and following whitespace
    // * 3  three components
    // = 12
    // + 1 for an extra white space after each pixel
    // = 13
    num_row_bytes = width * 13;
    break;

  default: break;
  }

  rv += num_row_bytes * height;

  return rv;
}

} // end namespace firevision
