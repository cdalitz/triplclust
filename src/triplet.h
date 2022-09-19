//
// triplet.h
//     Classes and functions for triplets of three points and
//     computing their dissimilarity.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#ifndef TRIPLET_H
#define TRIPLET_H

#include <limits>
#include <vector>

#include "pointcloud.h"

// triplet of three points
struct triplet {
  size_t point_index_a;
  size_t point_index_b;
  size_t point_index_c;
  Point center;
  Point direction;
  double error;
  friend bool operator<(const triplet &t1, const triplet &t2) {
    return (t1.error < t2.error);
  };
};

// dissimilarity for triplets.
// scale is an external scale factor.
class ScaleTripletMetric {
 private:
  double scale;

 public:
  ScaleTripletMetric(double s);
  double operator()(const triplet &lhs, const triplet &rhs);
};

// generates triplets from PointCloud
void generate_triplets(const PointCloud &cloud, std::vector<triplet> &triplets,
                       size_t k, size_t n, double a);
#endif
