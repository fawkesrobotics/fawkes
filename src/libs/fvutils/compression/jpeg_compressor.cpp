
/***************************************************************************
 *  jpeg_compressor.cpp - JPEG image compressor
 *
 *  Created: Sat Aug 12 13:42:39 2006 (in LFI of Central Medical Library
 *                                     of Germany, Cologne)
 *  Copyright  2005-2011  Tim Niemueller [www.niemueller.de]
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


#include <fvutils/compression/jpeg_compressor.h>
#include <fvutils/compression/jpeg_compressor_libjpeg.h>
#ifdef HAVE_MMAL
#  include <fvutils/compression/jpeg_compressor_mmal.h>
#endif

#include <core/exception.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class JpegImageCompressor <fvutils/compression/jpeg_compressor.h>
 * Jpeg image compressor.
 * The compressor can choose from several actual implementations. The default
 * is to use the system's version of libjpeg. On the Raspberry Pi the MMAL
 * implementation (which uses VideoCore) is preferred.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param quality JPEG quality in percent
 * @param jcs Jpeg colorspace
 */
JpegImageCompressor::JpegImageCompressor(unsigned int quality, JpegColorspace jcs)
{
  impl_ = 0;
#ifdef HAVE_MMAL
  if (jcs != JPEG_CS_RGB) {
    throw Exception("JpegImageCompressor MMAL can only encode to RGB colorspace");
  }
  impl_ = new JpegImageCompressorMMAL(quality);
#else
  #ifndef HAVE_LIBJPEG
  throw Exception("No JPEG compressor implementation available.");
  #else
  impl_ = new JpegImageCompressorLibJpeg(quality, jcs);
  #endif
#endif
}


/** Constructor.
 * @param impl_type force usage of this implementation type
 * @param quality JPEG quality in percent
 * @param jcs Jpeg colorspace
 */
JpegImageCompressor::JpegImageCompressor(JpegCompressorImplementation impl_type,
					 unsigned int quality, JpegColorspace jcs)
{
  impl_ = 0;
  if (impl_type == JPEG_CI_MMAL) {
#ifndef HAVE_MMAL
    throw Exception("JpegImageCompressor MMAL not available at compile time");
#else
  if (jcs != JPEG_CS_RGB) {
    throw Exception("JpegImageCompressor MMAL can only encode to RGB colorspace");
  }
  impl_ = new JpegImageCompressorMMAL(quality);
#endif
  } else if (impl_type == JPEG_CI_LIBJPEG) {
#ifndef HAVE_LIBJPEG
    throw Exception("No JPEG compressor implementation available.");
#else
    impl_ = new JpegImageCompressorLibJpeg(quality, jcs);
#endif
  } else {
    throw Exception("JpegImageCompressor: requested unknown implementation");
  }
}

/** Destructor. */
JpegImageCompressor::~JpegImageCompressor()
{
  delete impl_;
}

} // end namespace firevision
