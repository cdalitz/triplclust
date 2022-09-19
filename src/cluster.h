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

// compute hierarchical clustering
void compute_hc(const PointCloud &cloud, cluster_group &result,
                const std::vector<triplet> &triplets, double s, double t,
                bool tauto = false, double dmax = 0, bool is_dmax = false,
                Linkage method = SINGLE, int opt_verbose = 0);
// remove all small clusters
void cleanup_cluster_group(cluster_group &cg, size_t m, int opt_verbose = 0);
// convert the triplet indices ind *cl_group* to point indices.
void cluster_triplets_to_points(const std::vector<triplet> &triplets,
                                cluster_group &cl_group);
// adds the cluster ids to the points in *cloud*
void add_clusters(PointCloud &cloud, cluster_group &cl_group,
                  bool gnuplot = false);
#endif
