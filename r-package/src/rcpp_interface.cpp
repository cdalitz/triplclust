#include <Rcpp.h>
#include "pointcloud.h"
#include "triplclust.h"

// [[Rcpp::export]]
Rcpp::List triplclust_rcpp(
    Rcpp::NumericMatrix points,
    double r, bool r_dnn,
    int k, int n, double a,
    double s, bool s_dnn,
    double t, bool tauto,
    double dmax, bool dmax_dnn, bool is_dmax,
    std::string linkage, int m, int verbose, bool ordered) {
  PointCloud cloud;
  cloud.setOrdered(ordered);
  for (int row = 0; row < points.nrow(); ++row) {
    cloud.push_back(Point(points(row, 0), points(row, 1), points(row, 2)));
  }

  TriplClustParameters params;
  params.r = r;
  params.r_dnn = r_dnn;
  params.k = k;
  params.n = n;
  params.a = a;
  params.s = s;
  params.s_dnn = s_dnn;
  params.t = t;
  params.tauto = tauto;
  params.dmax = dmax;
  params.dmax_dnn = dmax_dnn;
  params.is_dmax = is_dmax;
  params.linkage = (linkage == "single") ? SINGLE :
                   (linkage == "complete") ? COMPLETE : AVERAGE;
  params.m = m;
  params.verbose = verbose;

  const cluster_group result = triplclust(cloud, params);
  Rcpp::List clusters(result.size());
  for (size_t cluster = 0; cluster < result.size(); ++cluster) {
    Rcpp::IntegerVector indices(result[cluster].size());
    for (size_t index = 0; index < result[cluster].size(); ++index) {
      indices[index] = static_cast<int>(result[cluster][index]) + 1;
    }
    clusters[cluster] = indices;
  }
  return clusters;
}