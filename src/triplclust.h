//
// triplclust.hpp
//     Library interface for the TriplClust algorithm.
//

#ifndef TRIPLCLUST_H
#define TRIPLCLUST_H

#include <cstddef>

#include "cluster.h"
#include "pointcloud.h"

struct TriplClustParameters {
  // Radius for point smoothing [2dNN]
  // (can be numeric or multiple of dNN)
  double r;
  bool r_dnn;  // interpret r as a multiple of dNN
  // Number of neighbours in triplet creation [19]
  size_t k;
  // Number of the best triplets to use [2]
  size_t n;
  // Maximum value for the angle between the triplet branches [0.03]
  double a;
  // Scaling factor for clustering [0.33dNN]
  // (can be numeric or multiple of dNN)
  double s;
  bool s_dnn;  // interpret s as a multiple of dNN
  // Best cluster distance [0.0]
  double t;
  // Flag indicating if t is set to 'auto' [true]
  bool tauto;
  // Maximum gap width within a triplet [none]
  // (can be numeric, multiple of dNN or 'none')
  double dmax;
  bool dmax_dnn;  // interpret dmax as a multiple of dNN
  // Flag indicating if dmax was set [false]
  bool is_dmax;
  // Linkage method for clustering [single]
  // (can be 'single', 'complete', 'average')
  Linkage linkage;
  // Minimum number of triplets for a cluster [5]
  size_t m;
  // Verbosity level [0]
  int verbose;

  TriplClustParameters()
      : r(2.0), r_dnn(false), k(19), n(2), a(0.03), s(0.33), s_dnn(false),
        t(0.0),tauto(true),
        dmax(0.0), dmax_dnn(false), is_dmax(false),
        linkage(SINGLE), m(5), verbose(0) {}
};

/*
 * Function for the TriplClust algorithm.
 @param cloud The input point cloud to be clustered.
 @param parameters The parameters for the TriplClust algorithm. 
 @returns a cluster_group containing the clusters found in the input point cloud.
 */
cluster_group triplclust(const PointCloud &cloud,
                         const TriplClustParameters &parameters);

#endif