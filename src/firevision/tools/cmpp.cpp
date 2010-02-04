
/***************************************************************************
 *  cmpp.cpp - Colormap Postprocessor. Extends the regions in the colormap
 *
 *  Created: Tue April 23 17:42:14 2009
 *  Copyright  2009  Daniel Beck
 *             2009  Stefan Schiffer
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

#include <utils/system/argparser.h>
#include <fvutils/colormap/colormap.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/colormap/cmfile.h>

#include <cstring>
#include <cmath>
#include <cstdio>

using namespace fawkes;
using namespace firevision;

int main( int argc, char** argv )
{
  ArgumentParser* argp = new ArgumentParser( argc, argv, "i:o:" );

  char* in_file  = NULL;
  char* out_file = NULL;

  if ( argp->has_arg( "i" ) )
  {
    in_file = strdup( argp->arg( "i" ) );
  }

  if ( argp->has_arg( "o" ) )
  {
    out_file = strdup( argp->arg( "o" ) );
  }

  if ( !in_file || !out_file )
  {
    printf("Usage: argv[0] -i <input colormap> -o <output colormap>\n");
  }
  else
  {
    printf("Reading colormap from file %s.\n", in_file);
    printf("Writing modified colormap to file %s.\n", out_file);
    
    ColormapFile cmfile;
    cmfile.read( in_file );
    Colormap *cm = cmfile.get_colormap();
    unsigned int cm_width  = cm->width();
    unsigned int cm_height = cm->height();
    unsigned int cm_depth  = cm->depth();

    unsigned char* cm_buffer = (unsigned char*) malloc( cm->size() );
    memcpy( (void*) cm_buffer, cm->get_buffer(), cm->size() );

    YuvColormap* cmpp = new YuvColormap( cm_depth, cm_width, cm_height );
    cmpp->set( cm_buffer );

    for ( unsigned int d = 0; d < cm_depth; ++d )
    {
      for ( unsigned int w = 0; w < cm_width; ++w )
      {
	for ( unsigned int h = 0; h < cm_height; ++h )
	{
	  float yuvfac = cm->deepness() / (float) cm->depth();
	  unsigned int y = (unsigned int) (d * yuvfac);
	  
	  color_t cur_color = cm->determine(y, w, h);

	  // skip current cell if it already has a color
	  if ( cur_color != C_OTHER )
	  { continue; }

	  unsigned int cm_counter[ C_OTHER + 1 ];

	  for ( unsigned int i = 0; i <= C_OTHER; ++i )
	  { cm_counter[ i ] = 0; }

	  unsigned int tst_radius_dp = 1;
	  unsigned int tst_radius_uv = 4;

	  unsigned int num_neighbours = 0;

	  for ( unsigned int dd = (unsigned int) fmax(d - tst_radius_dp, 0); 
		dd <= fmin( d + tst_radius_dp, cm_depth - 1);
		++dd ) 
	  {
	    for ( unsigned int ww = (unsigned int) fmax(w - tst_radius_uv, 0);
		  ww <= fmin( w + tst_radius_uv, cm_width - 1 );
		  ++ww )
	    {
	      for ( unsigned int hh = (unsigned int) fmax(h - tst_radius_uv, 0);
		    hh <= fmin( h + tst_radius_uv, cm_height - 1);
		    ++hh ) 
	      {
		color_t cur_color = cm->determine( (unsigned int) (dd * yuvfac), ww, hh );
		++cm_counter[ cur_color ];
		
		++num_neighbours;
	      }
	    }
	  }
	  
	  unsigned int max  = 0;
	  color_t max_color = C_OTHER;
	  
	  for ( unsigned int i = 0; i < C_OTHER; ++i )
	  {
	    if ( cm_counter[ i ] > max )
	    { 
	      max = cm_counter[ i ];
	      max_color = (color_t) i;
	    }
	  }
	  
	  if ( max > num_neighbours * 0.1 && max_color != C_OTHER )
	  {
	    printf("max=%d  max_color=%d  num_neighbours=%d\n", max, max_color, num_neighbours);
 	    cmpp->set( y, w, h, max_color );
	  }
	}  // h
      }  // w
    }  // d

    ColormapFile cmout( cm_depth, cm_width, cm_height );
    cmout.add_colormap( cmpp );
    printf( "Writing modified colormap.\n" );
    cmout.write( out_file );
  }

  return 0;
}
