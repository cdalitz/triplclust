//
// triplet.cpp
//     Classes and functions for triplets of three points and
//     computing their dissimilarity.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2024-02-02
// License: see ../LICENSE
//

#include <algorithm>
#include <cmath>

#include "kdtree/kdtree.hpp"
#include "triplet.h"


//-------------------------------------------------------------------
// Generates triplets from the PointCloud *cloud*.
// The resulting triplets are returned in *triplets*. *k* is the number
// of neighbores from a point, which are used for triplet generation.
// *n* is the number of the best triplet candidates to use. This can
// be lesser than *n*. *a* is the max error (1-angle) for the triplet
// to be a triplet candidate. If the cloud is ordered, only triplets
// with a.index < b.index < c.index are considered.
//-------------------------------------------------------------------
void generate_triplets(const PointCloud &cloud, std::vector<triplet> &triplets,
                       size_t k, size_t n, double a) {
  std::vector<double> distances;
  Kdtree::KdNodeVector nodes, result;
  std::vector<size_t> indices;  // save the indices so that they can be used
                                // for the KdNode constructor
  indices.resize(cloud.size(), 0);

  // build kdtree
  for (size_t i = 0; i < cloud.size(); ++i) {
    indices[i] = i;
    Kdtree::KdNode n = Kdtree::KdNode(cloud[i].as_vector(), (void *)&indices[i]);
    n.index = cloud[i].index;
    nodes.push_back(n);//, NULL, (int)cloud[i].index);
  }
  Kdtree::KdTree kdtree(&nodes);

  for (size_t point_index_b = 0; point_index_b < cloud.size();
       ++point_index_b) {
    distances.clear();
    Point point_b = cloud[point_index_b];
    std::vector<triplet> triplet_candidates;
    kdtree.k_nearest_neighbors(cloud[point_index_b].as_vector(), k, &result,
                               &distances);

    for (size_t result_index_a = 1; result_index_a < result.size();
         ++result_index_a) {
      // When the distance is 0, we have the same point as point_b
      if (distances[result_index_a] == 0) continue;
      Point point_a(result[result_index_a].point);
      point_a.index = result[result_index_a].index;
      if (cloud.isOrdered() && (point_a.index > point_b.index)) continue;
      size_t point_index_a = *(size_t *)result[result_index_a].data;

      Point direction_ab = point_b - point_a;
      double ab_norm = direction_ab.norm();
      direction_ab = direction_ab / ab_norm;

      for (size_t result_index_c = result_index_a + 1;
           result_index_c < result.size(); ++result_index_c) {
        // When the distance is 0, we have the same point as point_b
        if (distances[result_index_c] == 0) continue;
        Point point_c = Point(result[result_index_c].point);
        point_c.index = result[result_index_c].index;
        if (cloud.isOrdered() && (point_b.index > point_c.index)) continue;
        size_t point_index_c = *(size_t *)result[result_index_c].data;   

        Point direction_bc = point_c - point_b;
        double bc_norm = direction_bc.norm();
        direction_bc = direction_bc / bc_norm;

        const double angle = direction_ab * direction_bc;

        // calculate error
        const double error = 1.0f - angle;

        if (error <= a) {
          // calculate center
          Point center = (point_a + point_b + point_c) / 3.0f;

          // calculate direction
          Point direction = point_c - point_b;
          direction = direction / direction.norm();

          triplet new_triplet;

          new_triplet.point_index_a = point_index_a;
          new_triplet.point_index_b = point_index_b;
          new_triplet.point_index_c = point_index_c;
          new_triplet.center = center;
          new_triplet.direction = direction;
          new_triplet.error = error;

          triplet_candidates.push_back(new_triplet);
        }
      }
    }

    // order triplet candidates
    std::sort(triplet_candidates.begin(), triplet_candidates.end());

    // use the n best candidates
    for (size_t i = 0; i < std::min(n, triplet_candidates.size()); ++i) {
      triplets.push_back(triplet_candidates[i]);
    }
  }
}

// initialization of scale factor for triplet dissimilarity
ScaleTripletMetric::ScaleTripletMetric(double s) {
  this->scale = s;
}


// dissimilarity measure for triplets
double ScaleTripletMetric::operator()(const triplet &lhs, const triplet &rhs) {
  const double perpendicularDistanceA =
    (rhs.center - lhs.center +
     lhs.direction * (lhs.center - rhs.center) * lhs.direction).squared_norm();
  const double perpendicularDistanceB =
    (lhs.center - rhs.center +
     rhs.direction * (rhs.center - lhs.center) * rhs.direction).squared_norm();
  
  double anglecos = lhs.direction * rhs.direction;
  if (anglecos > 1.0) anglecos = 1.0;
  if (anglecos < -1.0) anglecos = -1.0;
  if (std::fabs(anglecos) < 1.0e-8) {
    return 1.0e+8;
  } else {
    return (
       std::sqrt(std::max(perpendicularDistanceA, perpendicularDistanceB)) /
       this->scale +
       std::fabs(std::tan(std::acos(anglecos))) );
  }
}
