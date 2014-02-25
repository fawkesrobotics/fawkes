
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
#include <fvutils/color/yuvrgb.h>
#include <fvutils/color/rgbyuv.h>

#include <core/exception.h>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <setjmp.h>
extern "C" {
#include <jpeglib.h>
#include <jerror.h>
}

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

///@cond INTERNALS

/** JPEG error manager type. */
typedef struct {
  struct jpeg_error_mgr pub;	/**< manager */
  jmp_buf setjmp_buffer;	/**< jmp buffer */
} fv_jpeg_error_mgr_t;


/** Defines a new destination manager to store images in memory
 * derived by jdatadst.c
 */
typedef struct {
  struct jpeg_destination_mgr pub;      /**< public fields */
  JOCTET *buffer;                       /**< start of buffer */
  int bufsize;                          /**< buffer size */
  int datacount;                        /**< final data size */
} fv_jpeg_memory_destination_mgr_t;


/** Initialize destination
 * called by jpeg_start_compress before any data is actually written.
 * @param cinfo compression info
 */
METHODDEF(void)
fv_jpeg_init_destination (j_compress_ptr cinfo)
{
  fv_jpeg_memory_destination_mgr_t *dest = (fv_jpeg_memory_destination_mgr_t *) cinfo->dest;
  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = dest->bufsize;
  dest->datacount=0;
}

/** Empty the output buffer
 * called whenever buffer fills up.
 * @param cinfo compression info
 */
METHODDEF(boolean)
fv_jpeg_empty_output_buffer (j_compress_ptr cinfo)
{
  fv_jpeg_memory_destination_mgr_t *dest = (fv_jpeg_memory_destination_mgr_t *) cinfo->dest;
  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = dest->bufsize;

  return TRUE;
}

/** Terminate destination
 * called by jpeg_finish_compress after all data has been written.
 * Usually needs to flush buffer.
 * @param cinfo compression info
 */
METHODDEF(void)
fv_jpeg_term_destination (j_compress_ptr cinfo)
{
  /* expose the finale compressed image size */

  fv_jpeg_memory_destination_mgr_t *dest = (fv_jpeg_memory_destination_mgr_t *) cinfo->dest;
  dest->datacount = dest->bufsize - dest->pub.free_in_buffer;
}

/** Setup memory destination.
 * @param cinfo compression info
 * @param buffer buffer
 * @param bufsize buffer size
 */
GLOBAL(void)
fv_jpeg_memory_destination_setup(j_compress_ptr cinfo, JOCTET *buffer,int bufsize)
{
  fv_jpeg_memory_destination_mgr_t *dest;
  if ( cinfo->dest == NULL ) {
    cinfo->dest = (struct jpeg_destination_mgr *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
                                  sizeof(fv_jpeg_memory_destination_mgr_t));
    
  }

  dest = (fv_jpeg_memory_destination_mgr_t *) cinfo->dest;
  dest->bufsize = bufsize;
  dest->buffer = buffer;
  dest->pub.init_destination = fv_jpeg_init_destination;
  dest->pub.empty_output_buffer = fv_jpeg_empty_output_buffer;
  dest->pub.term_destination = fv_jpeg_term_destination;
}

METHODDEF(void)
init_source(j_decompress_ptr cinfo)
{
  /* nothing to do */
}

METHODDEF(boolean)
fill_input_buffer(j_decompress_ptr cinfo)
{
  /* can't fill */
  return FALSE; 
}

METHODDEF(void)
skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
  if ((size_t)num_bytes > cinfo->src->bytes_in_buffer) {
    cinfo->src->next_input_byte = NULL;
    cinfo->src->bytes_in_buffer = 0;
  } else {
    cinfo->src->next_input_byte += (size_t) num_bytes;
    cinfo->src->bytes_in_buffer -= (size_t) num_bytes;
  }
}

METHODDEF(void)
term_source(j_decompress_ptr cinfo)
{
  /* nothing to do */
}

METHODDEF(void)
fv_jpeg_error_exit(j_common_ptr cinfo)
{
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  fv_jpeg_error_mgr_t *myerr = (fv_jpeg_error_mgr_t *) cinfo->err;
  
  /* Return control to the setjmp point */
  longjmp(myerr->setjmp_buffer, 1);
}


/**
 * set momory-jpeg image to JPEG lib Info struct
 * @param cinfo  JPEG lib decompress infomation structure
 * @param ptr    JPEG image 
 * @param size   JPEG image size
 */
GLOBAL(void)
fv_jpeg_memory_source_setup(j_decompress_ptr cinfo, unsigned char *ptr, size_t size)
{
    struct jpeg_source_mgr *src;
    src = cinfo->src = (struct jpeg_source_mgr *)
      (*cinfo->mem->alloc_small) ((j_common_ptr)cinfo, 
				  JPOOL_PERMANENT,
				  sizeof(*src));
    src->init_source       = init_source;
    src->fill_input_buffer = fill_input_buffer;
    src->skip_input_data   = skip_input_data;
    src->resync_to_restart = jpeg_resync_to_restart;
    src->term_source       = term_source;
    src->next_input_byte   = ptr;
    src->bytes_in_buffer   = size;
}

/// @endcond

/** @class JpegImageCompressorLibJpeg <fvutils/compression/jpeg_compressor.h>
 * Jpeg image compressor.
 */


/** Constructor.
 * @param quality JPEG quality in percent
 * @param jcs Jpeg colorspace
 */
JpegImageCompressorLibJpeg::JpegImageCompressorLibJpeg(unsigned int quality,
						       JpegImageCompressor::JpegColorspace jcs)
{
  this->quality = quality;
  jpeg_cs = jcs;
}

/** Destructor. */
JpegImageCompressorLibJpeg::~JpegImageCompressorLibJpeg()
{
}


bool
JpegImageCompressorLibJpeg::supports_vflip()
{
  return true;
}

void
JpegImageCompressorLibJpeg::set_vflip(bool enabled)
{
  vflip = enabled;
}

void
JpegImageCompressorLibJpeg::compress()
{
  struct jpeg_compress_struct cinfo;
  fv_jpeg_error_mgr_t jerr;
  unsigned int row_stride;
  unsigned char *row_buffer;

  // mem destination specific
  fv_jpeg_memory_destination_mgr_t *dest;

  // file destination specific
  FILE *outfile = NULL;

  /* zero out the compression info structure and
     allocate a new compressor handle */
  memset (&cinfo, 0, sizeof(cinfo));
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = fv_jpeg_error_exit;

  /* Establish the setjmp return context for my_error_exit to use. */
  if (setjmp(jerr.setjmp_buffer)) {
    char buffer[JMSG_LENGTH_MAX];
    (*cinfo.err->format_message) ((jpeg_common_struct *)&cinfo, buffer);

    /* If we get here, the JPEG code has signaled an error.
     * We need to clean up the JPEG object, close the input file, and return.
     */
    jpeg_destroy_compress(&cinfo);
    throw fawkes::Exception("Compression failed: %s", buffer);
  }

  jpeg_create_compress(&cinfo);
 
  /* Setup JPEG datastructures */
  cinfo.image_width = width;      /* image width and height, in pixels */
  cinfo.image_height = height;
  cinfo.input_components = 3;     /* # of color components per pixel=3 RGB */
  if ( jpeg_cs == JpegImageCompressor::JPEG_CS_RGB ) {
    cinfo.in_color_space = JCS_RGB;
  } else {
    cinfo.in_color_space = JCS_YCbCr;
  }

  row_stride = cinfo.image_width * cinfo.input_components;

  if ( compdest == COMP_DEST_MEM ) {
    // mem
    fv_jpeg_memory_destination_setup(&cinfo, (JOCTET *)jpeg_buffer, jpeg_buffer_size);
  } else {
    outfile = fopen(filename, "wb");
    if (outfile == NULL) {
      throw fawkes::Exception("JpegImageCompressorLibJpeg: cannot open %s\n", filename);
    }
    jpeg_stdio_dest( &cinfo, outfile );
  }
  /* Setup compression */
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality (&cinfo, quality, true /* limit to baseline-JPEG values */);
  jpeg_start_compress(&cinfo, true);

  /* compress each scanline one-at-a-time */
  row_buffer = (unsigned char *)malloc( row_stride );


  if ( jpeg_cs == JpegImageCompressor::JPEG_CS_RGB ) {
    if (vflip) {
      while (cinfo.next_scanline < cinfo.image_height) {
	convert_line_yuv422planar_to_rgb(
	  buffer, row_buffer,
	  cinfo.image_width, cinfo.image_height,
	  cinfo.image_height - cinfo.next_scanline - 1, 0);
	jpeg_write_scanlines(&cinfo, &row_buffer, 1);
      }
    } else {
      while (cinfo.next_scanline < cinfo.image_height) {
	convert_line_yuv422planar_to_rgb(
          buffer, row_buffer,
	  cinfo.image_width, cinfo.image_height,
	  cinfo.next_scanline, 0 );
	jpeg_write_scanlines(&cinfo, &row_buffer, 1);
      }
    }
  } else {
    if (vflip) {
      while (cinfo.next_scanline < cinfo.image_height) {
	convert_line_yuv422planar_to_yuv444packed(
          buffer, row_buffer,
	  cinfo.image_width, cinfo.image_height,
	  cinfo.image_height - cinfo.next_scanline - 1, 0 );
	jpeg_write_scanlines(&cinfo, &row_buffer, 1);
      }
    } else {
      while (cinfo.next_scanline < cinfo.image_height) {
	convert_line_yuv422planar_to_yuv444packed(
          buffer, row_buffer,
	  cinfo.image_width, cinfo.image_height,
	  cinfo.next_scanline, 0 );
	jpeg_write_scanlines(&cinfo, &row_buffer, 1);
      }
    }
  }

  free(row_buffer);
  jpeg_finish_compress(&cinfo);

  if ( compdest == COMP_DEST_MEM ) {
    /* Now extract the size of the compressed buffer */
    dest=(fv_jpeg_memory_destination_mgr_t *)cinfo.dest;
    jpeg_bytes = dest->datacount; /* the actual compressed datasize */
  } else {
    fclose( outfile );
  }

  /* destroy the compressor handle */
  jpeg_destroy_compress(&cinfo);

}


void
JpegImageCompressorLibJpeg::set_image_dimensions(unsigned int width, unsigned int height)
{
  this->width = width;
  this->height = height;
}


void
JpegImageCompressorLibJpeg::set_image_buffer(colorspace_t cspace, unsigned char *buffer)
{
  if ( cspace == YUV422_PLANAR ) {
    this->buffer = buffer;
  }
}


void
JpegImageCompressorLibJpeg::set_compression_destination(ImageCompressor::CompressionDestination cd)
{
  compdest = cd;
}


bool
JpegImageCompressorLibJpeg::supports_compression_destination(ImageCompressor::CompressionDestination cd)
{
  return true;
}


void
JpegImageCompressorLibJpeg::set_destination_buffer(unsigned char *buf, unsigned int buf_size)
{
  jpeg_buffer = buf;
  jpeg_buffer_size = buf_size;
}


size_t
JpegImageCompressorLibJpeg::compressed_size()
{
  return jpeg_bytes;
}

size_t
JpegImageCompressorLibJpeg::recommended_compressed_buffer_size()
{
  return width * height / 4;
}


void
JpegImageCompressorLibJpeg::set_filename(const char *filename)
{
  this->filename = filename;
}

} // end namespace firevision
