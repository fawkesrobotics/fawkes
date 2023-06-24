/***************************************************************************
 *  pf_draw.c: Particle filter; drawing routines
 *
 *  Created: Thu May 24 18:39:07 2012
 *  Copyright  2000  Brian Gerkey
 *             2000  Kasper Stoy
 *             2012  Tim Niemueller [www.niemueller.de]
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

/*  From:
 *  Player - One Hell of a Robot Server (LGPL)
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 */
/**************************************************************************
 * Desc: Particle filter; drawing routines
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 *************************************************************************/

#ifdef INCLUDE_RTKGUI

#	include "pf.h"
#	include "pf_kdtree.h"
#	include "pf_pdf.h"
#	include "rtk.h"

#	include <assert.h>
#	include <math.h>
#	include <stdlib.h>

/// @cond EXTERNAL

// Draw the statistics
void pf_draw_statistics(pf_t *pf, rtk_fig_t *fig);

// Draw the sample set
void
pf_draw_samples(pf_t *pf, rtk_fig_t *fig, int max_samples)
{
	int              i;
	pf_sample_set_t *set;

	set         = pf->sets + pf->current_set;
	max_samples = MIN(max_samples, set->sample_count);

	for (i = 0; i < max_samples; i++) {
		pf_sample_t *sample = set->samples + i;

		double px = sample->pose.v[0];
		double py = sample->pose.v[1];
		double pa = sample->pose.v[2];

		//printf("%f %f\n", px, py);

		rtk_fig_point(fig, px, py);
		rtk_fig_arrow(fig, px, py, pa, 0.1, 0.02);
		//rtk_fig_rectangle(fig, px, py, 0, 0.1, 0.1, 0);
	}

	return;
}

// Draw the hitogram (kd tree)
void
pf_draw_hist(pf_t *pf, rtk_fig_t *fig)
{
	pf_sample_set_t *set;

	set = pf->sets + pf->current_set;

	rtk_fig_color(fig, 0.0, 0.0, 1.0);
	pf_kdtree_draw(set->kdtree, fig);

	return;
}

// Draw the CEP statistics
void
pf_draw_cep_stats(pf_t *pf, rtk_fig_t *fig)
{
	pf_vector_t mean;
	double      var;

	pf_get_cep_stats(pf, &mean, &var);
	var = sqrt(var);

	rtk_fig_color(fig, 0, 0, 1);
	rtk_fig_ellipse(fig, mean.v[0], mean.v[1], mean.v[2], 3 * var, 3 * var, 0);

	return;
}

// Draw the cluster statistics
void
pf_draw_cluster_stats(pf_t *pf, rtk_fig_t *fig)
{
	int              i;
	pf_sample_set_t *set;
	pf_vector_t      mean;
	pf_matrix_t      cov;
	pf_matrix_t      r, d;

	set = pf->sets + pf->current_set;

	for (i = 0; i < set->cluster_count; i++) {
		pf_cluster_t *cluster = set->clusters + i;

		//weight = cluster->weight;
		mean = cluster->mean;
		cov  = cluster->cov;

		// Compute unitary representation S = R D R^T
		pf_matrix_unitary(&r, &d, cov);

		/* Debugging
    printf("mean = \n");
    pf_vector_fprintf(mean, stdout, "%e");
    printf("cov = \n");
    pf_matrix_fprintf(cov, stdout, "%e");
    printf("r = \n");
    pf_matrix_fprintf(r, stdout, "%e");
    printf("d = \n");
    pf_matrix_fprintf(d, stdout, "%e");
    */

		// Compute the orientation of the error ellipse (first eigenvector)
		double o  = atan2(r.m[1][0], r.m[0][0]);
		double d1 = 6 * sqrt(d.m[0][0]);
		double d2 = 6 * sqrt(d.m[1][1]);

		if (d1 > 1e-3 && d2 > 1e-3) {
			// Draw the error ellipse
			rtk_fig_ellipse(fig, mean.v[0], mean.v[1], o, d1, d2, 0);
			rtk_fig_line_ex(fig, mean.v[0], mean.v[1], o, d1);
			rtk_fig_line_ex(fig, mean.v[0], mean.v[1], o + M_PI / 2, d2);
		}

		// Draw a direction indicator
		rtk_fig_arrow(fig, mean.v[0], mean.v[1], mean.v[2], 0.50, 0.10);
		rtk_fig_arrow(fig, mean.v[0], mean.v[1], mean.v[2] + 3 * sqrt(cov.m[2][2]), 0.50, 0.10);
		rtk_fig_arrow(fig, mean.v[0], mean.v[1], mean.v[2] - 3 * sqrt(cov.m[2][2]), 0.50, 0.10);
	}

	return;
}

/// @endcond

#endif
