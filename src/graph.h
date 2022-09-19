//
// graph.h
//     Classes and functions for computing the MST and for
//     splitting up clusters at gaps
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#ifndef MSD_H
#define MSD_H
#include <cstddef>
#include <vector>

#include "pointcloud.h"

// Split *cluster* in multiple new clusters and return the result in
// *new_clusters". The mst of the cluster is created and all edges are
// removed with a wheigth > *dmax*. The connected comonents are computed
// and returned as new clusters if their size is >= *min_size*.
void max_step(std::vector<std::vector<size_t> > &new_clusters,
              const std::vector<size_t> &cluster, const PointCloud &cloud,
              double dmax, size_t min_size);

#endif
