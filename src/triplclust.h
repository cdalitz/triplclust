//
// triplclust.hpp
//     Library interface for the TriplClust algorithm.
//

#ifndef TRIPLCLUST_H
#define TRIPLCLUST_H

#include <cstddef>
#include <vector>

#include "cluster.h"
#include "pointcloud.h"

// Values that are resolved during the run and are only reported for
// debugging and reproducibility. They are not used by the algorithm itself.
struct TriplClustDebugInfo {
  // absolute characteristic length dNN that was used for scaling the
  // parameters. 0 if no parameter depended on dNN.
  double dnn;
  // absolute cluster distance threshold that was effectively applied
  double t;
  // true if that threshold was determined automatically instead of being
  // given by the user
  bool t_auto;
  // number of clusters removed by pruning
  size_t pruned_clusters;

  TriplClustDebugInfo()
      : dnn(0.0), t(0.0), t_auto(false), pruned_clusters(0) {}
};

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
  // resolved absolute values, only reported for debugging
  TriplClustDebugInfo debug;

  TriplClustParameters()
      : r(2.0), r_dnn(false), k(19), n(2), a(0.03), s(0.33), s_dnn(false),
        t(0.0),tauto(true),
        dmax(0.0), dmax_dnn(false), is_dmax(false),
        linkage(SINGLE), m(5), verbose(0), debug() {}
};

/*
 * Function for the TriplClust algorithm.
 @param cloud The input point cloud to be clustered.
 @param parameters The parameters for the TriplClust algorithm. On return,
 this struct holds the parameters that were effectively used: every value
 given as a multiple of dNN is resolved to an absolute value and its
 corresponding *_dnn flag is cleared. On an exception the struct is left
 unchanged.
 @param cdist If not NULL, it receives the distance of each of the n-1 merge
 steps of the triplet clustering, where n is the number of generated triplets.
 This is the same data that the CLI writes to debug_cdist.csv for -vv.
 @returns a cluster_group containing the clusters found in the input point cloud.
 */
cluster_group triplclust(const PointCloud &cloud,
                         TriplClustParameters &parameters,
                         std::vector<double> *cdist = NULL);

#endif