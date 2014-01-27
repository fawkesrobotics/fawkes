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

/* This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 */

#ifndef __FIREVISION_UTILS_COLOR_THRESHOLD_H_
#define __FIREVISION_UTILS_COLOR_THRESHOLD_H_

#include <math.h>
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
}

#endif /* __FIREVISION_UTILS_COLOR_THRESHOLD_H_ */
