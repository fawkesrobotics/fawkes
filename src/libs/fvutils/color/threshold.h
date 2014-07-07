/***************************************************************************
 * threshold.h - Determine whether a given YUV chroma value is "similar" to
 * a certain reference color, according to defined chroma and saturation
 * thresholds.
 *
 * The algorithm is ported from the VLC colorthreshold filter written by
 * Sigmund Augdal and Antoine Cellerier. Cf.
 * modules/video_filter/colorthres.c in the VLC source tree.
 *
 * Initially ported in 2014 by Victor Mataré.
 *
 * The original code is licensed under GPL 2.1, so we do the same.
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

#ifndef __FIREVISION_UTILS_COLOR_THRESHOLD_H_
#define __FIREVISION_UTILS_COLOR_THRESHOLD_H_

#include <cmath>
#include <sys/types.h>

namespace firevision {

/** Determine if two colors are similar - ATTENTION:
 * All u/v values have to be normalized to -127..128, NOT the usual 0..255!
 * Otherwise the similarity calculation won't work.
 * @param u YUV observed color U value
 * @param v YUV observed color V value
 * @param ref_u YUV reference color U value
 * @param ref_v YUV reference color V value
 * @param ref_length Length of the (rev_u, ref_v) vector in 2D chroma space.
 * Caller should precompute this for performance reasons (pythagoras!).
 * @param chroma_thresh Maximum chroma difference
 * @param sat_thresh Minimum saturation
 * @return true if color (u, v) is similar to (ref_u, ref_v), false otherwise.
 */
inline bool is_similar(int u, int v, int ref_u, int ref_v, int ref_length, int chroma_thresh, int sat_thresh) {
  int length = sqrt(u * u + v * v);

  int diffu, diffv;
  int64_t difflen2, thres;

  diffu = ref_u * length - u * ref_length;
  diffv = ref_v * length - v * ref_length;
  difflen2 = diffu * diffu + diffv * diffv;
  thres = length * ref_length;
  thres *= thres;

  return (length > sat_thresh) && (difflen2 * chroma_thresh < thres);
}

inline bool is_similar_y(int y, int u, int v,
  int ref_y, int ref_u, int ref_v,
  int ref_length, int chroma_thresh, int sat_thresh, int y_thresh) {

  return is_similar(u, v, ref_u, ref_v, ref_length, chroma_thresh, sat_thresh)
#if defined(__GNUC__) && ((__GNUC__ == 4 && __GNUC_MINOR__ >= 6) || (__GNUC__ > 4))
      && std::abs(y - ref_y) < (255 - y_thresh);
#else
      && ((y-ref_y) < 0 ? -1*(y-ref_y) : (y-ref_y)) < (255 - y_thresh);
#endif
}
}

#endif /* __FIREVISION_UTILS_COLOR_THRESHOLD_H_ */
