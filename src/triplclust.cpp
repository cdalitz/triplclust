//
// triplclust.cpp
//     Library implementation of the TriplClust algorithm.
//

#include "triplclust.h"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "dnn.h"
#include "graph.h"
#include "output.h"
#include "triplet.h"


bool validate_parameters( const PointCloud          &cloud,
                            const TriplClustParameters &par,
                            std::string                &errMsg )
{
    // ---- 0) Cloud -------------------------------------------------------
    if (cloud.empty()) {
        errMsg = "[triplclust] input point cloud is empty";
        return false;
    }

    // ---- 1) radius ------------------------------------------------------
    if (par.r <= 0.0) {
        errMsg = "[triplclust] radius (-r) must be > 0";
        return false;
    }

    // ---- 2) neighbour counts --------------------------------------------
    if (par.k == 0) {
        errMsg = "[triplclust] k (-k) must be >= 1";
        return false;
    }
    if (par.n == 0) {
        errMsg = "[triplclust] n (-n) must be >= 1";
        return false;
    }
    if (par.n > par.k) {
        errMsg = "[triplclust] n (-n) cannot be larger than k (-k)";
        return false;
    }

    // ---- 3) angle -------------------------------------------------------
    constexpr double PI = 3.1415926535897932384626433;
    if (par.a <= 0.0 || par.a >= PI) {
        errMsg = "[triplclust] alpha (-a) must be in (0, π)";
        return false;
    }

    // ---- 4) scaling factor -----------------------------------------------
    if (par.s <= 0.0) {
        errMsg = "[triplclust] scale (-s) must be > 0";
        return false;
    }

    // ---- 5) explicit clustering distance ----------------------------------
    if (!par.tauto && par.t < 0.0) {
        errMsg = "[triplclust] explicit cluster distance (-t) must be >= 0";
        return false;
    }

    // ---- 6) max‑gap (dmax) -----------------------------------------------
    if (par.is_dmax && par.dmax < 0.0) {
        errMsg = "[triplclust] dmax (-dmax) must be >= 0 when enabled";
        return false;
    }

    // ---- 7) linkage method -----------------------------------------------
    switch (par.linkage) {
        case SINGLE:
        case COMPLETE:
        case AVERAGE:
            break;   // OK
        default:
            errMsg = "[triplclust] unknown linkage method (-link)";
            return false;
    }

    // ---- 8) minimum triplets per cluster ---------------------------------
    if (par.m == 0) {
        errMsg = "[triplclust] minimum cluster size (-m) must be >= 1";
        return false;
    }

    // ---- 9) verbosity ----------------------------------------------------
    if (par.verbose < 0) {
        errMsg = "[triplclust] verbosity level (-v / -vv) cannot be negative";
        return false;
    }

    // ---- 10) optional sanity warnings (non‑fatal) -----------------------
    // These are just *advice*; they do not cause failure.
    if (par.r > 1e6) {
        std::cerr << "[triplclust] warning: radius is unusually large ("
                  << par.r << ")\n";
    }

    // All checks passed
    errMsg.clear();
    return true;
}             

cluster_group triplclust(const PointCloud &cloud,
                         const TriplClustParameters &parameters) {
    TriplClustParameters scaled_parameters = parameters;
  std::string validationError;
  if (!validate_parameters(cloud, parameters, validationError)) {
        throw std::invalid_argument(validationError);
  }

    const bool needs_dnn = parameters.r_dnn || parameters.s_dnn ||
                                                 (parameters.is_dmax && parameters.dmax_dnn);
    if (needs_dnn) {
        if (cloud.size() < 2) {
            throw std::invalid_argument(
                    "[triplclust] at least two points are required to compute dNN");
        }
        const double dnn = std::sqrt(first_quartile(cloud));
        if (!std::isfinite(dnn) || dnn <= 0.0) {
            throw std::invalid_argument(
                    "[triplclust] dNN computed as zero. Remove duplicate points.");
        }
        if (parameters.r_dnn) scaled_parameters.r *= dnn;
        if (parameters.s_dnn) scaled_parameters.s *= dnn;
        if (parameters.is_dmax && parameters.dmax_dnn) {
            scaled_parameters.dmax *= dnn;
        }
    }
    scaled_parameters.r_dnn = false;
    scaled_parameters.s_dnn = false;
    scaled_parameters.dmax_dnn = false;

    if (!validate_parameters(cloud, scaled_parameters, validationError)) {
        throw std::invalid_argument(validationError);
    }

  // Step 1) smoothing by position averaging of neighboring points
  PointCloud cloud_smooth;
    smoothen_cloud(cloud, cloud_smooth, scaled_parameters.r);

    if (scaled_parameters.verbose > 1) {
    bool rc = cloud_to_csv(cloud_smooth);
    if (!rc)
      std::cerr << "[Error] can't write debug_smoothed.csv" << std::endl;
    rc = debug_gnuplot(cloud, cloud_smooth);
    if (!rc)
      std::cerr << "[Error] can't write debug_smoothed.gnuplot" << std::endl;
  }

  // Step 2) finding triplets of approximately collinear points
  std::vector<triplet> triplets;
    generate_triplets(cloud_smooth, triplets, scaled_parameters.k,
                                        scaled_parameters.n, scaled_parameters.a);

  // Step 3) single link hierarchical clustering of the triplets
  cluster_group cl_group;
    compute_hc(cloud_smooth, cl_group, triplets, scaled_parameters.s,
                         scaled_parameters.t, scaled_parameters.tauto,
                         scaled_parameters.dmax, scaled_parameters.is_dmax,
                         scaled_parameters.linkage, scaled_parameters.verbose);

  // Step 4) pruning by removal of small clusters
    cleanup_cluster_group(cl_group, scaled_parameters.m,
                                                scaled_parameters.verbose);
  cluster_triplets_to_points(triplets, cl_group);

  // Optionally split up clusters at gaps > dmax
    if (scaled_parameters.is_dmax) {
    cluster_group cleaned_up_cluster_group;
    for (cluster_group::iterator cl = cl_group.begin(); cl != cl_group.end();
         ++cl) {
    max_step(cleaned_up_cluster_group, *cl, cloud, scaled_parameters.dmax,
           scaled_parameters.m + 2);
    }
    cl_group = cleaned_up_cluster_group;
  }

  return cl_group;
}