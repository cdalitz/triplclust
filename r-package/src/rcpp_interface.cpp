#include <Rcpp.h>
#include <vector>
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
    std::string linkage, int m, bool ordered) {
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
  params.verbose = 0;

  // keep the requested values, triplclust() resolves params in place
  const TriplClustParameters requested = params;

  // distance of each of the n-1 merge steps of the triplet clustering
  std::vector<double> cdist;
  cluster_group cl_group = triplclust(cloud, params, &cdist);
  add_clusters(cloud, cl_group, false);

  // The plain names are the values that were requested (e.g. r = 2 with
  // r_dnn = TRUE for "2dNN"), the _real suffix holds the value that was
  // actually used, with all dNN multiples resolved to absolute numbers.
  const char *linkage_name = (params.linkage == SINGLE) ? "single" :
                             (params.linkage == COMPLETE) ? "complete" :
                             "average";
  Rcpp::List used_params = Rcpp::List::create(
      Rcpp::Named("r") = requested.r,
      Rcpp::Named("r_dnn") = requested.r_dnn,
      Rcpp::Named("r_real") = params.r,
      Rcpp::Named("k") = static_cast<int>(params.k),
      Rcpp::Named("n") = static_cast<int>(params.n),
      Rcpp::Named("a") = params.a,
      Rcpp::Named("s") = requested.s,
      Rcpp::Named("s_dnn") = requested.s_dnn,
      Rcpp::Named("s_real") = params.s,
      Rcpp::Named("t") = requested.t,
      Rcpp::Named("tauto") = requested.tauto,
      Rcpp::Named("t_real") = params.debug.t,
      Rcpp::Named("dmax") = requested.dmax,
      Rcpp::Named("dmax_dnn") = requested.dmax_dnn,
      Rcpp::Named("is_dmax") = requested.is_dmax,
      Rcpp::Named("dmax_real") = params.dmax,
      Rcpp::Named("dnn_real") = params.debug.dnn,
      Rcpp::Named("linkage") = linkage_name,
      Rcpp::Named("m") = static_cast<int>(params.m),
      Rcpp::Named("verbose") = params.verbose
  );

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

  // merge distances of the triplet clustering
  Rcpp::NumericVector cdist_vec(cdist.size());
  for (size_t i = 0; i < cdist.size(); ++i) {
    cdist_vec[static_cast<R_xlen_t>(i)] = cdist[i];
  }

  return Rcpp::List::create(
      Rcpp::Named("labels") = point_clusters,
      Rcpp::Named("params") = used_params,
      Rcpp::Named("cdist") = cdist_vec
  );
}