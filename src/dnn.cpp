//
// dnn.cpp
//     Functions for computing the characteristic length dnn
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#include <algorithm>
#include <numeric>
#include <vector>

#include "dnn.h"
#include "kdtree/kdtree.hpp"

//-------------------------------------------------------------------
// Compute mean squared distances.
// the distances is computed for every point in *cloud* to its *k*
// nearest neighbours. The distances are returned in *msd*.
//-------------------------------------------------------------------
void compute_mean_square_distance(const PointCloud &cloud,
                                  std::vector<double> &msd, int k) {
  // compute mean square distances for every point to its k nearest neighbours
  Kdtree::KdNodeVector nodes, result;
  double sum;

  // build kdtree
  for (size_t i = 0; i < cloud.size(); ++i) {
    nodes.push_back(cloud[i].as_vector());
  }
  Kdtree::KdTree kdtree(&nodes);

  k++;  // k must be one higher because the first point found by the kdtree is
        // the point itself

  for (size_t i = 0; i < cloud.size(); ++i) {
    // compute
    std::vector<double> squared_distances;
    kdtree.k_nearest_neighbors(cloud[i].as_vector(), k, &result,
                               &squared_distances);

    squared_distances.erase(squared_distances.begin());  // The first value
                                                         // must be deleted
                                                         // because it is
                                                         // the distance
                                                         // with the point
                                                         // itself

    sum = std::accumulate(squared_distances.begin(), squared_distances.end(),
                          0.0);
    msd.push_back(sum / squared_distances.size());
  }
}

//-------------------------------------------------------------------
// Compute first quartile of the mean squared distance of all points
// in *cloud*
//-------------------------------------------------------------------
double first_quartile(const PointCloud &cloud) {
  std::vector<double> msd;
  compute_mean_square_distance(cloud, msd, 1);
  const double q1 = msd.size() / 4;
  std::nth_element(msd.begin(), msd.begin() + q1, msd.end());
  return msd[q1];
}
