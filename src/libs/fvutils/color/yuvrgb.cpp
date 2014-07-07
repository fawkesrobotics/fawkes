
/****************************************************************************
 *  yuvrgb.h - YUV to RGB conversion - specific methods, macros and constants
 *
 *  Created: Sat Aug 12 15:02:41 2006
 *  based on colorspaces.h from Tue Feb 23 13:49:38 2005
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/color/yuvrgb.h>
#include <core/macros.h>

#include <fvutils/cpu/mmx.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** YUV to RGB Conversion
 * B = 1.164(Y - 16)                  + 2.018(U - 128)
 * G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128)
 * R = 1.164(Y - 16) + 1.596(V - 128)
 *
 * Values have to be clamped to keep them in the [0-255] range.
 * Rumour has it that the valid range is actually a subset of [0-255] (fourcc.org mentions an RGB range
 * of [16-235] mentioned) but clamping the values into [0-255] seems to produce acceptable results.
 * @param YUV unsigned char array that contains the pixels, 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param RGB where the RGB output will be written to, will have pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param width Width of the image contained in the YUV buffer
 * @param height Height of the image contained in the YUV buffer
 */
void
yuv411packed_to_rgb_plainc(const unsigned char *YUV, unsigned char *RGB,
			   unsigned int width, unsigned int height)
{
  int y0, y1, y2, y3, u, v;
  unsigned int i = 0;
  while (i < (width * height)*3/2) {
    u  = YUV[i++] - 128;
    y0 = YUV[i++] -  16;
    y1 = YUV[i++] -  16;
    v  = YUV[i++] - 128;
    y2 = YUV[i++] -  16;
    y3 = YUV[i++] -  16;

    // Set red, green and blue bytes for pixel 0
    *RGB++ = clip( (76284 * y0 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y0 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y0 + 132252 * u             ) >> 16 );

    // Set red, green and blue bytes for pixel 1
    *RGB++ = clip( (76284 * y1 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y1 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y1 + 132252 * u             ) >> 16 );

    // Set red, green and blue bytes for pixel 2
    *RGB++ = clip( (76284 * y2 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y2 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y2 + 132252 * u             ) >> 16 );

    // Set red, green and blue bytes for pixel 3
    *RGB++ = clip( (76284 * y3 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y3 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y3 + 132252 * u             ) >> 16 );

  }
}


/** YUV to RGB Conversion
 * B = 1.164(Y - 16)                  + 2.018(U - 128)
 * G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128)
 * R = 1.164(Y - 16) + 1.596(V - 128)
 *
 * Values have to be clamped to keep them in the [0-255] range.
 * Rumour has it that the valid range is actually a subset of [0-255] (fourcc.org mentions an RGB range
 * of [16-235] mentioned) but clamping the values into [0-255] seems to produce acceptable results.
 * @param YUV unsigned char array that contains the pixels, 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param RGB where the RGB output will be written to, will have pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param width Width of the image contained in the YUV buffer
 * @param height Height of the image contained in the YUV buffer
 */
void
yuv422planar_to_rgb_plainc(const unsigned char *planar, unsigned char *RGB, unsigned int width, unsigned int height)
{

  short y1, y2, u, v;
  const unsigned char *yp, *up, *vp;
  unsigned int i;

  yp = planar;
  up = planar + (width * height);
  vp = up + (width * height / 2);

  for (i = 0; i < (width * height / 2); ++i) {

    y1 = *yp++;
    y2 = *yp++;
    u  = *up++;
    v  = *vp++;

    y1 -=  16;
    y2 -=  16;
    u  -= 128;
    v  -= 128;

    // Set red, green and blue bytes for pixel 0
    *RGB++ = clip( (76284 * y1 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y1 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y1 + 132252 * u             ) >> 16 );

    // Set red, green and blue bytes for pixel 1
    *RGB++ = clip( (76284 * y2 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y2 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y2 + 132252 * u             ) >> 16 );

  }
}



/** YUV to RGB Conversion
 * B = 1.164(Y - 16)                  + 2.018(U - 128)
 * G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128)
 * R = 1.164(Y - 16) + 1.596(V - 128)
 *
 * Values have to be clamped to keep them in the [0-255] range.
 * Rumour has it that the valid range is actually a subset of [0-255] (fourcc.org mentions an RGB range
 * of [16-235] mentioned) but clamping the values into [0-255] seems to produce acceptable results.
 * @param YUV unsigned char array that contains the pixels, 4 pixels in 8 byte macro pixel, line after
 *            line
 * @param RGB where the RGB output will be written to, will have pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param width Width of the image contained in the YUV buffer
 * @param height Height of the image contained in the YUV buffer
 */
void
yuv422packed_to_rgb_plainc(const unsigned char *YUV, unsigned char *RGB,
			   const unsigned int width, const unsigned int height)
{
  int y0, y1, u, v;
  unsigned int i = 0;
  for (unsigned int pixel = 0; pixel < (width * height); pixel += 2) {
    u  = YUV[i++] - 128;
    y0 = YUV[i++] -  16;
    v  = YUV[i++] - 128;
    y1 = YUV[i++] -  16;

    // Set red, green and blue bytes for pixel 0
    *RGB++ = clip( (76284 * y0 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y0 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y0 + 132252 * u             ) >> 16 );

    // Set red, green and blue bytes for pixel 1
    *RGB++ = clip( (76284 * y1 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y1 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y1 + 132252 * u             ) >> 16 );
  }
}

/** Convert YUV422 planar to BGR.
 * Use formula in aforementioned function.
 * @param YUV YUV422 planar buffer
 * @param BGR BGR buffer
 * @param width Width of the image contained in the YUV buffer
 * @param height Height of the image contained in the YUV buffer
 */
void
yuv422planar_to_bgr_plainc(const unsigned char *planar, unsigned char *BGR,
			   unsigned int width, unsigned int height)
{

  short y1, y2, u, v;
  const unsigned char *yp, *up, *vp;
  unsigned int i;

  yp = planar;
  up = planar + (width * height);
  vp = up + (width * height / 2);

  for (i = 0; i < (width * height / 2); ++i) {

    y1 = *yp++;
    y2 = *yp++;
    u  = *up++;
    v  = *vp++;

    y1 -=  16;
    y2 -=  16;
    u  -= 128;
    v  -= 128;

    // Set red, green and blue bytes for pixel 0
    *BGR++ = clip( (76284 * y1 + 132252 * u             ) >> 16 );
    *BGR++ = clip( (76284 * y1 -  25625 * u - 53281 * v ) >> 16 );
    *BGR++ = clip( (76284 * y1 + 104595 * v             ) >> 16 );

    // Set red, green and blue bytes for pixel 1
    *BGR++ = clip( (76284 * y2 + 132252 * u             ) >> 16 );
    *BGR++ = clip( (76284 * y2 -  25625 * u - 53281 * v ) >> 16 );
    *BGR++ = clip( (76284 * y2 + 104595 * v             ) >> 16 );
  }
}


void
yuv422planar_to_rgb_with_alpha_plainc(const unsigned char *planar, unsigned char *RGB, unsigned int width, unsigned int height)
{

  short y1, y2, u, v;
  const unsigned char *yp, *up, *vp;
  unsigned int i;

  yp = planar;
  up = planar + (width * height);
  vp = up + (width * height / 2);

  for (i = 0; i < (width * height / 2); ++i) {

    y1 = *yp++;
    y2 = *yp++;
    u  = *up++;
    v  = *vp++;

    y1 -=  16;
    y2 -=  16;
    u  -= 128;
    v  -= 128;

    // Set red, green and blue bytes for pixel 0
    *RGB++ = clip( (76284 * y1 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y1 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y1 + 132252 * u             ) >> 16 );
    *RGB++ = 255;

    // Set red, green and blue bytes for pixel 1
    *RGB++ = clip( (76284 * y2 + 104595 * v             ) >> 16 );
    *RGB++ = clip( (76284 * y2 -  25625 * u - 53281 * v ) >> 16 );
    *RGB++ = clip( (76284 * y2 + 132252 * u             ) >> 16 );
    *RGB++ = 255;

  }

}


void
yuv422planar_to_bgr_with_alpha_plainc(const unsigned char *planar, unsigned char *BGR, unsigned int width, unsigned int height)
{

  short y1, y2, u, v;
  const unsigned char *yp, *up, *vp;
  unsigned int i;

  yp = planar;
  up = planar + (width * height);
  vp = up + (width * height / 2);

  for (i = 0; i < (width * height / 2); ++i) {

    y1 = *yp++;
    y2 = *yp++;
    u  = *up++;
    v  = *vp++;

    y1 -=  16;
    y2 -=  16;
    u  -= 128;
    v  -= 128;

    // Set red, green and blue bytes for pixel 0
    *BGR++ = clip( (76284 * y1 + 132252 * u             ) >> 16 );
    *BGR++ = clip( (76284 * y1 -  25625 * u - 53281 * v ) >> 16 );
    *BGR++ = clip( (76284 * y1 + 104595 * v             ) >> 16 );
    *BGR++ = 255;

    // Set red, green and blue bytes for pixel 1
    *BGR++ = clip( (76284 * y2 + 132252 * u             ) >> 16 );
    *BGR++ = clip( (76284 * y2 -  25625 * u - 53281 * v ) >> 16 );
    *BGR++ = clip( (76284 * y2 + 104595 * v             ) >> 16 );
    *BGR++ = 255;

  }

}


void
yuv422packed_to_bgr_with_alpha_plainc(const unsigned char *YUV, unsigned char *BGR,
				      unsigned int width, unsigned int height)
{

  int y0, y1, u, v;
  unsigned int i = 0;
  while (i < (width * height * 2)) {
    u  = YUV[i++] - 128;
    y0 = YUV[i++] -  16;
    v  = YUV[i++] - 128;
    y1 = YUV[i++] -  16;

    // Set red, green and blue bytes for pixel 0
    *BGR++ = clip( (76284 * y0 + 132252 * u             ) >> 16 );
    *BGR++ = clip( (76284 * y0 -  25625 * u - 53281 * v ) >> 16 );
    *BGR++ = clip( (76284 * y0 + 104595 * v             ) >> 16 );
    *BGR++ = 255;

    // Set red, green and blue bytes for pixel 1
    *BGR++ = clip( (76284 * y1 + 132252 * u             ) >> 16 );
    *BGR++ = clip( (76284 * y1 -  25625 * u - 53281 * v ) >> 16 );
    *BGR++ = clip( (76284 * y1 + 104595 * v             ) >> 16 );
    *BGR++ = 255;

  }
}


#if ( \
	 defined __i386__ || \
	 defined __386__ || \
	 defined __X86__ || \
	 defined _M_IX86 || \
	 defined i386)

#define CRV    104595
#define CBU    132251
#define CGU    25624
#define CGV    53280
#define YMUL   76283
#define OFF    32768
#define BITRES 16

/* calculation float resolution in bits */
/* ie RES = 6 is 10.6 fixed point */
/*    RES = 8 is 8.8 fixed point */
/*    RES = 4 is 12.4 fixed point */
/* NB: going above 6 will lead to overflow... :( */
#define RES    6

#define RZ(i)  (i >> (BITRES - RES))
#define FOUR(i) {i, i, i, i}

__aligned(8) const volatile unsigned short _const_crvcrv[4] = FOUR(RZ(CRV));
__aligned(8) const volatile unsigned short _const_cbucbu[4] = FOUR(RZ(CBU));
__aligned(8) const volatile unsigned short _const_cgucgu[4] = FOUR(RZ(CGU));
__aligned(8) const volatile unsigned short _const_cgvcgv[4] = FOUR(RZ(CGV));
__aligned(8) const volatile unsigned short _const_ymul  [4] = FOUR(RZ(YMUL));
__aligned(8) const volatile unsigned short _const_128   [4] = FOUR(128);
__aligned(8) const volatile unsigned short _const_32    [4] = FOUR(RZ(OFF));
__aligned(8) const volatile unsigned short _const_16    [4] = FOUR(16);

#define CONST_CRVCRV *_const_crvcrv
#define CONST_CBUCBU *_const_cbucbu
#define CONST_CGUCGU *_const_cgucgu
#define CONST_CGVCGV *_const_cgvcgv
#define CONST_YMUL   *_const_ymul
#define CONST_128    *_const_128
#define CONST_32     *_const_32
#define CONST_16     *_const_16

void
yuv411planar_to_rgb_mmx (const unsigned char *yuv, unsigned char *rgb,
			 unsigned int w, unsigned int h)
{
  unsigned int xx, yy;
  const unsigned char *yp1, *up, *vp;
  unsigned char *dp1;

  /* plane pointers */
  yp1 = yuv;
  up = yuv + (w * h);
  vp = up + (w * (h / 4));
  /* destination pointers */
  dp1 = rgb;



  yp1 = yuv;
  up = yuv + (w * h);
  vp = up + ((w / 2) * (h / 2));
  dp1 = rgb;
  for (yy = 0; yy < h; yy++)
    {
      for (xx = 0; xx < w; xx += 8)
	{
	  movq_m2r(*yp1, mm0);
	  movq_r2r(mm0, mm1);
	  psrlw_i2r(8, mm0);
	  psllw_i2r(8, mm1);
	  psrlw_i2r(8, mm1);

	  pxor_r2r(mm7, mm7);
	  movd_m2r(*up, mm3);
	  movd_m2r(*vp, mm2);

	  punpcklbw_r2r(mm7, mm2);
	  punpcklbw_r2r(mm7, mm3);

	  movq_m2r(CONST_16, mm4);
	  psubsw_r2r(mm4, mm0);
	  psubsw_r2r(mm4, mm1);

	  movq_m2r(CONST_128, mm5);
	  psubsw_r2r(mm5, mm2);
	  psubsw_r2r(mm5, mm3);

	  movq_m2r(CONST_YMUL, mm4);
	  pmullw_r2r(mm4, mm0);
	  pmullw_r2r(mm4, mm1);

	  movq_m2r(CONST_CRVCRV, mm7);
	  pmullw_r2r(mm3, mm7);

	  movq_m2r(CONST_CBUCBU, mm6);
	  pmullw_r2r(mm2, mm6);

	  movq_m2r(CONST_CGUCGU, mm5);
	  pmullw_r2r(mm2, mm5);

	  movq_m2r(CONST_CGVCGV, mm4);
	  pmullw_r2r(mm3, mm4);

	  movq_r2r(mm0, mm2);
	  paddsw_r2r(mm7, mm2);
	  paddsw_r2r(mm1, mm7);

	  psraw_i2r(RES, mm2);
	  psraw_i2r(RES, mm7);
	  packuswb_r2r(mm7, mm2);

	  pxor_r2r(mm7, mm7);
	  movq_r2r(mm2, mm3);
	  punpckhbw_r2r(mm7, mm2);
	  punpcklbw_r2r(mm3, mm7);
	  por_r2r(mm7, mm2);

	  movq_r2r(mm0, mm3);
	  psubsw_r2r(mm5, mm3);
	  psubsw_r2r(mm4, mm3);
	  paddsw_m2r(CONST_32, mm3);

	  movq_r2r(mm1, mm7);
	  psubsw_r2r(mm5, mm7);
	  psubsw_r2r(mm4, mm7);
	  paddsw_m2r(CONST_32, mm7);

	  psraw_i2r(RES, mm3);
	  psraw_i2r(RES, mm7);
	  packuswb_r2r(mm7, mm3);

	  pxor_r2r(mm7, mm7);
	  movq_r2r(mm3, mm4);
	  punpckhbw_r2r(mm7, mm3);
	  punpcklbw_r2r(mm4, mm7);
	  por_r2r(mm7, mm3);

	  movq_m2r(CONST_32, mm4);
	  paddsw_r2r(mm6, mm0);
	  paddsw_r2r(mm6, mm1);
	  paddsw_r2r(mm4, mm0);
	  paddsw_r2r(mm4, mm1);
	  psraw_i2r(RES, mm0);
	  psraw_i2r(RES, mm1);
	  packuswb_r2r(mm1, mm0);

	  pxor_r2r(mm7, mm7);
	  movq_r2r(mm0, mm5);
	  punpckhbw_r2r(mm7, mm0);
	  punpcklbw_r2r(mm5, mm7);
	  por_r2r(mm7, mm0);

	  pxor_r2r(mm1, mm1);
	  movq_r2r(mm0, mm5);
	  movq_r2r(mm3, mm6);
	  movq_r2r(mm2, mm7);
	  punpckhbw_r2r(mm3, mm2);
	  punpcklbw_r2r(mm6, mm7);
	  punpckhbw_r2r(mm1, mm0);
	  punpcklbw_r2r(mm1, mm5);

	  movq_r2r(mm7, mm1);
	  punpckhwd_r2r(mm5, mm7);
	  punpcklwd_r2r(mm5, mm1);

	  movq_r2r(mm2, mm4);
	  punpckhwd_r2r(mm0, mm2);
	  punpcklwd_r2r(mm0, mm4);

	  movntq_r2m(mm1, *(dp1));
	  movntq_r2m(mm7, *(dp1 + 8));
	  movntq_r2m(mm4, *(dp1 + 16));
	  movntq_r2m(mm2, *(dp1 + 24));

	  yp1 += 8;
	  up += 4;
	  vp += 4;
	  dp1 += 8 * 4;
	}
      if (yy & 0x1)
	{
	  up -= w / 2;
	  vp -= w / 2;
	}
    }
  emms();
}
#endif

} // end namespace firevision
