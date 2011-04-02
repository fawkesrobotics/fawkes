
/***************************************************************************
 *  imagedecompressor.h - image de-compressor interface
 *
 *  Created: July 2007 (Sci-Bono, South Africa, B&B)
 *  Copyright  2006-2007  Daniel Beck
 *             2007-2011  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/color/conversions.h>
#include <fvutils/compression/jpeg_decompressor.h>
#include <core/exception.h>

#include <sys/types.h>
#include <cstdio>
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

typedef struct {
  struct jpeg_source_mgr pub;

  JOCTET * buffer;
} my_source_mgr;

typedef my_source_mgr * my_src_ptr;


struct my_error_mgr {
  struct jpeg_error_mgr pub;	/* "public" fields */

  jmp_buf setjmp_buffer;	/* for return to caller */
};

typedef struct my_error_mgr * my_error_ptr;

/*
 * Here's the routine that will replace the standard error_exit method:
 */

METHODDEF(void)
my_error_exit (j_common_ptr cinfo)
{
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  /* Return control to the setjmp point */
  longjmp(myerr->setjmp_buffer, 1);
}

METHODDEF(void)
init_source (j_decompress_ptr cinfo)
{
}


METHODDEF(boolean)
fill_input_buffer (j_decompress_ptr cinfo)
{
  return TRUE;
}

METHODDEF(void)
skip_input_data (j_decompress_ptr cinfo, long num_bytes)
{
  my_src_ptr src = (my_src_ptr) cinfo->src;
  /* Just a dumb implementation for now.  Could use fseek() except
   * it doesn't work on pipes.  Not clear that being smart is worth
   * any trouble anyway --- large skips are infrequent.
   */
  if (num_bytes > 0) {
    while (num_bytes > (long) src->pub.bytes_in_buffer) {
      num_bytes -= (long) src->pub.bytes_in_buffer;
      (void) fill_input_buffer(cinfo);
      /* note we assume that fill_input_buffer will never return FALSE,
       * so suspension need not be handled.
       */
    }
    src->pub.next_input_byte += (size_t) num_bytes;
    src->pub.bytes_in_buffer -= (size_t) num_bytes;
  }
}

METHODDEF(void)
term_source (j_decompress_ptr cinfo)
{
  /* no work necessary here */
}

GLOBAL(void)
my_mem_src (j_decompress_ptr cinfo, JOCTET * buffer, size_t bytes)
{
  my_src_ptr src;

 if (cinfo->src == NULL) {	/* first time for this JPEG object? */
   cinfo->src = (struct jpeg_source_mgr *)
     (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				 sizeof(my_source_mgr));
   src = (my_src_ptr) cinfo->src;
//    src->buffer = (JOCTET *)
//      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
// 				 INPUT_BUF_SIZE * SIZEOF(JOCTET));
  }

  src = (my_src_ptr) cinfo->src;
  src->pub.init_source = init_source;
  src->pub.fill_input_buffer = fill_input_buffer;
  src->pub.skip_input_data = skip_input_data;
  src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
  src->pub.term_source = term_source;
  src->pub.bytes_in_buffer = bytes;
  src->pub.next_input_byte = buffer;
}

/// @endcond

/** @class JpegImageDecompressor <fvutils/compression/jpeg_decompressor.h>
 * Decompressor for JPEG images.
 * @author Daniel Beck
 * @author Tim Niemueller
 */

/** Constructor. */
JpegImageDecompressor::JpegImageDecompressor()
{
}

void
JpegImageDecompressor::decompress()
{
  JSAMPROW row_pointer[1];
  unsigned long location = 0;
  unsigned char *buffer;
	
  // JPEG decompression
  // Allocate and initialize a JPEG decompression object
  struct jpeg_decompress_struct cinfo;

  struct my_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = my_error_exit;
  /* Establish the setjmp return context for my_error_exit to use. */
  if (setjmp(jerr.setjmp_buffer)) {
    char buffer[JMSG_LENGTH_MAX];
    (*cinfo.err->format_message) ((jpeg_common_struct *)&cinfo, buffer);

    /* If we get here, the JPEG code has signaled an error.
     * We need to clean up the JPEG object, close the input file, and return.
     */
    jpeg_destroy_decompress(&cinfo);
    throw fawkes::Exception("Decompression failed: %s", buffer);
  }

  jpeg_create_decompress(&cinfo);

  // Specify the source of the compressed data
  my_mem_src(&cinfo, _compressed_buffer, _compressed_buffer_size);

  // Call jpeg_read_header() to obtain image info
  jpeg_read_header(&cinfo, TRUE);

  // set output color space
  //  cinfo.out_color_space = JCS_YCbCr;
  
  // Set parameters for decompression
  
  // jpeg_start_decompress(...);
  jpeg_start_decompress(&cinfo);

  buffer = (unsigned char*)malloc( cinfo.output_width * cinfo.output_height * cinfo.num_components );
  
  row_pointer[0] = (unsigned char *)malloc( cinfo.output_width * cinfo.num_components );
  
  // while (scan lines remain to be read)
  //   jpeg_read_scanlines(...);
  while( cinfo.output_scanline < cinfo.image_height )
    {
      jpeg_read_scanlines( &cinfo, row_pointer, 1 );
      for( unsigned int i=0; i < cinfo.image_width * cinfo.num_components; i++)
	buffer[location++] = row_pointer[0][i];
    }
  
  // jpeg_finish_decompress(...);
  jpeg_finish_decompress(&cinfo);
  
  // Release the JPEG decompression object
  jpeg_destroy_decompress(&cinfo);
  
  free(row_pointer[0]);

  // convert to yuv422packed and store in member frame_buffer
  convert(RGB, YUV422_PLANAR, buffer, _decompressed_buffer, cinfo.output_width, cinfo.output_height);

  free(buffer);

}

} // end namespace firevision
