//
// cluster.cpp
//     Functions for triplet clustering and for propagating
//     the triplet cluster labels to points
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2019-04-02
// License: see ../LICENSE
//

#include <algorithm>
#include <cmath>
#include <fstream>

#include "cluster.h"
#include "hclust/fastcluster.h"

// compute mean of *a* with size *m*
double mean(const double *a, size_t m) {
  double sum = 0;
  for (size_t i = 0; i < m; ++i) {
    sum += a[i];
  }
  return sum / m;
}

// compute standard deviation of *a* with size *m*
double sd(const double *a, size_t m) {
  double sum = 0, result;
  double mean_val = mean(a, m);
  for (size_t k = 0; k < m; ++k) {
    double tmp = mean_val - a[k];
    sum += (tmp * tmp);
  }
  result = (1.0 / (m - 1.0)) * sum;
  return std::sqrt(result);
}

//-------------------------------------------------------------------
// computation of condensed distance matrix.
// The distance matrix is computed from the triplets in *triplets*
// and saved in *result*. *triplet_metric* is used as distance metric.
//-------------------------------------------------------------------
void calculate_distance_matrix(const std::vector<triplet> &triplets,
                               const PointCloud &cloud, double *result,
                               ScaleTripletMetric &triplet_metric) {
  size_t const triplet_size = triplets.size();
  size_t k = 0;

  for (size_t i = 0; i < triplet_size; ++i) {
    for (size_t j = i + 1; j < triplet_size; j++) {
      result[k++] = triplet_metric(triplets[i], triplets[j]);
    }
  }
}

//-------------------------------------------------------------------
// Computation of the clustering.
// The triplets in *triplets* are clustered by the fastcluster algorithm
// and the result is returned as cluster_group. *t* is the cut distance
// and *triplet_metric* is the distance metric for the triplets.
// *opt_verbose* is the verbosity level for debug outputs. the clustering
// is returned in *result*.
//-------------------------------------------------------------------
void compute_hc(const PointCloud &cloud, cluster_group &result,
                const std::vector<triplet> &triplets, double s, double t,
                bool tauto, double dmax, bool is_dmax, Linkage method,
                int opt_verbose) {
  const size_t triplet_size = triplets.size();
  size_t k, cluster_size;
  hclust_fast_methods link;

  if (!triplet_size) {
    // if no triplets are generated
    return;
  }
  // choose linkage method
  switch (method) {
    case SINGLE:
      link = HCLUST_METHOD_SINGLE;
      break;
    case COMPLETE:
      link = HCLUST_METHOD_COMPLETE;
      break;
    case AVERAGE:
      link = HCLUST_METHOD_AVERAGE;
      break;
  }

  double *distance_matrix = new double[(triplet_size * (triplet_size - 1)) / 2];
  double *cdists = new double[triplet_size - 1];
  int *merge = new int[2 * (triplet_size - 1)], *labels = new int[triplet_size];
  ScaleTripletMetric metric(s);
  calculate_distance_matrix(triplets, cloud, distance_matrix, metric);

  hclust_fast(triplet_size, distance_matrix, link, merge, cdists);

  // splitting the dendrogram into clusters
  if (tauto) {
    // automatic stopping criterion where cdist is unexpected large
    for (k = (triplet_size - 1) / 2; k < (triplet_size - 1); ++k) {
      if ((cdists[k - 1] > 0.0 || cdists[k] > 1.0e-8) &&
          (cdists[k] > cdists[k - 1] + 2 * sd(cdists, k + 1))) {
        break;
      }
    }
    if (opt_verbose) {
      double automatic_t;
      double prev_cdist = (k > 0) ? cdists[k - 1] : 0.0;
      if (k < (triplet_size - 1)) {
        automatic_t = (prev_cdist + cdists[k]) / 2.0;
      } else {
        automatic_t = cdists[k - 1];
      }
      std::cout << "[Info] optimal cdist threshold: " << automatic_t
                << std::endl;
    }
  } else {
    // fixed threshold t
    for (k = 0; k < (triplet_size - 1); ++k) {
      if (cdists[k] >= t) {
        break;
      }
    }
  }
  cluster_size = triplet_size - k;
  cutree_k(triplet_size, merge, cluster_size, labels);

  // generate clusters
  for (size_t i = 0; i < cluster_size; ++i) {
    cluster_t new_cluster;
    result.push_back(new_cluster);
  }

  for (size_t i = 0; i < triplet_size; ++i) {
    result[labels[i]].push_back(i);
  }

  if (opt_verbose > 1) {
    // write debug file
    const char *fname = "debug_cdist.csv";
    std::ofstream of(fname);
    of << std::fixed;  // set float style
    if (of.is_open()) {
      for (size_t i = 0; i < (triplet_size - 1); ++i) {
        of << cdists[i] << std::endl;
      }
    } else {
      std::cerr << "[Error] could not write file '" << fname << "'\n";
    }
    of.close();
  }

  // cleanup
  delete[] distance_matrix;
  delete[] cdists;
  delete[] merge;
  delete[] labels;
}

//-------------------------------------------------------------------
// Remove all clusters in *cl_group* which contains less then *m*
// triplets. *cl_group* will be modified.
//-------------------------------------------------------------------
void cleanup_cluster_group(cluster_group &cl_group, size_t m, int opt_verbose) {
  size_t old_size = cl_group.size();
  cluster_group::iterator it = cl_group.begin();
  while (it != cl_group.end()) {
    if (it->size() < m) {
      it = cl_group.erase(it);
    } else {
      ++it;
    }
  }
  if (opt_verbose > 0) {
    std::cout << "[Info] in pruning removed clusters: "
              << old_size - cl_group.size() << std::endl;
  }
}

//-------------------------------------------------------------------
// Convert the triplet indices ind *cl_group* to point indices.
// *triplets* contains all triplets and *cl_group* will be modified.
//-------------------------------------------------------------------
void cluster_triplets_to_points(const std::vector<triplet> &triplets,
                                cluster_group &cl_group) {
  for (size_t i = 0; i < cl_group.size(); ++i) {
    cluster_t point_indices, &current_cluster = cl_group[i];
    for (std::vector<size_t>::const_iterator triplet_index =
             current_cluster.begin();
         triplet_index < current_cluster.end(); ++triplet_index) {
      const triplet &current_triplet = triplets[*triplet_index];
      point_indices.push_back(current_triplet.point_index_a);
      point_indices.push_back(current_triplet.point_index_b);
      point_indices.push_back(current_triplet.point_index_c);
    }
    // sort point-indices and remove duplicates
    std::sort(point_indices.begin(), point_indices.end());
    std::vector<size_t>::iterator new_end =
        std::unique(point_indices.begin(), point_indices.end());
    point_indices.resize(std::distance(point_indices.begin(), new_end));
    // replace triplet cluster with point cluster
    current_cluster = point_indices;
  }
}

//-------------------------------------------------------------------
// Adds the cluster ids to the points in *cloud*
// *cl_group* contains the clusters with the point indices. For every
// point the corresponding cluster id is saved. For gnuplot the points
// which overlap between multiple clusteres are saved in seperate
// clusters in *cl_group*
//-------------------------------------------------------------------
void add_clusters(PointCloud &cloud, cluster_group &cl_group, bool gnuplot) {
  for (size_t i = 0; i < cl_group.size(); ++i) {
    const std::vector<size_t> &current_cluster = cl_group[i];
    for (std::vector<size_t>::const_iterator point_index =
             current_cluster.begin();
         point_index != current_cluster.end(); ++point_index) {
      cloud[*point_index].cluster_ids.insert(i);
    }
  }

  // add point indices to the corresponding cluster/vertex vector. this is for
  // the gnuplot output
  if (gnuplot) {
    std::vector<cluster_t> verticies;
    for (size_t i = 0; i < cloud.size(); ++i) {
      const Point &p = cloud[i];
      if (p.cluster_ids.size() > 1) {
        // if the point is in multiple clusters add it to the corresponding
        // vertex or create one if none exists
        bool found = false;
        for (std::vector<cluster_t>::iterator v = verticies.begin();
             v != verticies.end(); ++v) {
          if (cloud[v->at(0)].cluster_ids == p.cluster_ids) {
            v->push_back(i);
            found = true;
          }
        }
        if (!found) {
          cluster_t v;
          v.push_back(i);
          verticies.push_back(v);
        }
        // remove the point from all other clusters
        for (std::set<size_t>::iterator it = p.cluster_ids.begin();
             it != p.cluster_ids.end(); ++it) {
          cluster_t &cluster = cl_group[*it];
          cluster.erase(std::remove(cluster.begin(), cluster.end(), i),
                        cluster.end());
        }
      }
    }
    // extend clusters with verticies
    cl_group.reserve(cl_group.size() + verticies.size());
    cl_group.insert(cl_group.end(), verticies.begin(), verticies.end());
  }
}
