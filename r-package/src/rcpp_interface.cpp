#include <Rcpp.h>
#include "cluster.h"
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

  cluster_group cl_group = triplclust(cloud, params);
  add_clusters(cloud, cl_group, false);

  Rcpp::List point_clusters(cloud.size());
  for (size_t point = 0; point < cloud.size(); ++point) {
    Rcpp::IntegerVector cluster_ids(cloud[point].cluster_ids.size());
    size_t index = 0;
    for (std::set<size_t>::const_iterator cluster =
             cloud[point].cluster_ids.begin();
         cluster != cloud[point].cluster_ids.end(); ++cluster) {
      cluster_ids[index++] = static_cast<int>(*cluster) + 1;
    }
    point_clusters[point] = cluster_ids;
  }
  return point_clusters;
}