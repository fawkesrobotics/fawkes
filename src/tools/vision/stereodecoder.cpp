
/***************************************************************************
 *  stereodecoder.cpp - Stereo decoder utility
 *
 *  Created: Wed Jul 11 15:50:10 2007 (Atlanta Airport)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/bumblebee2.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/readers/fvraw.h>
#include <fvutils/color/conversions.h>

#include <list>
#include <string>
#include <cstdlib>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

using namespace std;
using namespace fawkes;
using namespace firevision;

/** Interleave to YUV422 planar buffers.
 * Creates an image buffer which has both images side by side.
 * @param yuv422_first first buffer
 * @param yuv
 * */
void
interleave_yuv422planar(unsigned char *yuv422_first, unsigned char *yuv422_second,
			unsigned char *out,
			unsigned int width, unsigned int height)
{
  unsigned char *y1, *y2, *yo, *u1, *u2, *uo, *v1, *v2, *vo;
  unsigned int half_width = width / 2;
  y1 = yuv422_first;
  u1 = y1 + width * height;
  v1 = u1 + (width * height / 2);
  y2 = yuv422_second;
  u2 = y2 + width * height;
  v2 = u2 + (width * height / 2);
  yo = out;
  uo = yo + width * height * 2;
  vo = uo + width * height;

  for ( unsigned int i = 0; i < height; ++i) {

    memcpy(yo, y1, width);
    yo += width;
    y1 += width;

    memcpy(yo, y2, width);
    yo += width;
    y2 += width;

    memcpy(uo, u1, half_width);
    uo += half_width;
    u1 += half_width;

    memcpy(uo, u2, half_width);
    uo += half_width;
    u2 += half_width;

    memcpy(vo, v1, half_width);
    vo += half_width;
    v1 += half_width;

    memcpy(vo, v2, half_width);
    vo += half_width;
    v2 += half_width;
  }

  /*
  unsigned int half_width = width / 2;
  for ( unsigned int i = 0; i < height; ++i) {
    memcpy(out, yuv422_first, half_width);
    out += half_width;
    yuv422_first += half_width;
    memcpy(out, yuv422_first, half_width);
    out += half_width;
    yuv422_second += half_width;
  }
  for ( unsigned int i = 0; i < height; ++i) {
    memcpy(out, yuv422_first, half_width);
    out += half_width;
    yuv422_first += half_width;
    memcpy(out, yuv422_first, half_width);
    out += half_width;
    yuv422_second += half_width;
  }
  */
}

int
main(int argc, char **argv)
{

  if ( argc < 2 ) {
    printf("Usage: %s <dir>\n", argv[0]);
    exit(-1);
  }

  string dirname = argv[1];

  // Get all files
  DIR *dir;
  struct dirent *dirp;

  list<string> files;

  if ( NULL == (dir = opendir(dirname.c_str())) ) {
    printf("Failed to open directory %s\n", dirname.c_str());
    exit(-2);
  }

  while ( NULL != (dirp = readdir(dir)) ) {
    if ( NULL != strstr(dirp->d_name, ".raw") ) {
      files.push_back(dirp->d_name);
    }
  }

  closedir(dir);

  files.sort();

  /*
  // create directories
  char *tmp;
  asprintf(&tmp, "%s/%s", dirname.c_str(), "orig_jpeg");
  mkdir(tmp, 0644);
  free(tmp);

  // create directories
  asprintf(&tmp, "%s/%s", dirname.c_str(), "disp_jpeg");
  mkdir(tmp, 0644);
  free(tmp);
  */

  JpegWriter *jpeg = new JpegWriter("tmp.jpg");

  // printf("%lu images to convert\n", files.size());

  unsigned int in = 0;
  try {
    for (list<string>::iterator f = files.begin(); f != files.end(); ++f) {
      FvRawReader *fvraw = new FvRawReader((dirname + "/" + (*f)).c_str());
      printf("%4u Converting %s (%s)  ", ++in, (dirname + "/" + (*f)).c_str(), colorspace_to_string(fvraw->colorspace()));
      unsigned char *raw16 = malloc_buffer(fvraw->colorspace(), fvraw->pixel_width(), fvraw->pixel_height() * 2);
      unsigned char *rgb = (unsigned char *)malloc(colorspace_buffer_size(RGB, fvraw->pixel_width(), fvraw->pixel_height()) * 2);
      unsigned char *deinterlaced = (unsigned char *)malloc(fvraw->pixel_width() * fvraw->pixel_height() * 2);
      unsigned char *yuv = (unsigned char *)malloc_buffer(YUV422_PLANAR, fvraw->pixel_width(), fvraw->pixel_height() * 2);
      unsigned char *yuv_interleaved = (unsigned char *)malloc_buffer(YUV422_PLANAR, fvraw->pixel_width(), fvraw->pixel_height() * 2);
      fvraw->set_buffer(raw16);
      fvraw->read();

      printf("(%ux%u)   ", fvraw->pixel_width(), fvraw->pixel_height());

      Bumblebee2Camera::deinterlace_stereo(raw16, deinterlaced,
					   fvraw->pixel_width(), fvraw->pixel_height());
      Bumblebee2Camera::decode_bayer(deinterlaced, rgb,
				     fvraw->pixel_width(), fvraw->pixel_height(),
				     BAYER_PATTERN_BGGR);
      /*
      convert(RGB, YUV422_PLANAR,
	      rgb + colorspace_buffer_size(RGB, fvraw->pixel_width(), fvraw->pixel_height()),
	      yuv + colorspace_buffer_size(YUV422_PLANAR, fvraw->pixel_width(), fvraw->pixel_height()),
	      fvraw->pixel_width(), fvraw->pixel_height());
      */
      convert(RGB, YUV422_PLANAR,
	      rgb,
	      yuv,
	      fvraw->pixel_width(), fvraw->pixel_height());

      convert(RGB, YUV422_PLANAR,
	      rgb + colorspace_buffer_size(RGB, fvraw->pixel_width(), fvraw->pixel_height()),
	      yuv + colorspace_buffer_size(YUV422_PLANAR, fvraw->pixel_width(), fvraw->pixel_height()),
	      fvraw->pixel_width(), fvraw->pixel_height());

      interleave_yuv422planar(yuv + colorspace_buffer_size(YUV422_PLANAR, fvraw->pixel_width(), fvraw->pixel_height()),
			      yuv, yuv_interleaved, fvraw->pixel_width(), fvraw->pixel_height());

      *f += ".jpg";
      printf("to %s\n", (dirname + "/orig_jpeg/" + (*f)).c_str());

      jpeg->set_filename((dirname + "/orig_jpeg/" + (*f)).c_str());
      jpeg->set_buffer(YUV422_PLANAR, yuv_interleaved);
      // jpeg->set_buffer(YUV422_PLANAR, yuv);
      jpeg->set_dimensions(fvraw->pixel_width() * 2, fvraw->pixel_height());
      jpeg->write();

      delete fvraw;
      free(raw16);
      free(rgb);
      free(deinterlaced);
      free(yuv);
      free(yuv_interleaved);
    }
  } catch (Exception &e) {
    e.print_trace();
    throw;
  }
}
