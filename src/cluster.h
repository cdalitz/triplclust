//
// cluster.h
//     Functions for triplet clustering and for propagating
//     the triplet cluster labels to points
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#ifndef CLUSTER_H
#define CLUSTER_H

#include <cstddef>
#include <vector>

#include "triplet.h"
#include "util.h"

typedef std::vector<size_t> cluster_t;

typedef std::vector<cluster_t> cluster_group;

// compute hierarchical clustering. If *t_used* is not NULL, it receives the
// cluster distance threshold that was effectively applied, i.e. the
// automatically determined one when *tauto* is true. If *cdist* is not NULL,
// it receives the distance of each of the n-1 merge steps.
void compute_hc(const PointCloud &cloud, cluster_group &result,
                const std::vector<triplet> &triplets, double s, double t,
                bool tauto = false, double dmax = 0, bool is_dmax = false,
                Linkage method = SINGLE, double *t_used = NULL,
                std::vector<double> *cdist = NULL);
// remove all small clusters and return how many clusters were removed
size_t cleanup_cluster_group(cluster_group &cg, size_t m);
// convert the triplet indices ind *cl_group* to point indices.
void cluster_triplets_to_points(const std::vector<triplet> &triplets,
                                cluster_group &cl_group);
// adds the cluster ids to the points in *cloud*
void add_clusters(PointCloud &cloud, cluster_group &cl_group,
                  bool gnuplot = false);
#endif
