//
// output.h
//     Functions for writing output files.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#ifndef OUTPUT_H
#define OUTPUT_H
#include "cluster.h"
#include "pointcloud.h"

// saves a PointCloud *cloud* as csv file.
bool cloud_to_csv(const PointCloud &cloud,
                  const char *fname = "debug_smoothed.csv");
// saves smoothen cloud as gnuplot script.
bool debug_gnuplot(const PointCloud &cloud, const PointCloud &cloud_smooth,
                   const char *fname = "debug_smoothed.gnuplot");
// prints gnuplot script to stdout.
void clusters_to_gnuplot(const PointCloud &cloud,
                         const std::vector<cluster_t> &clusters);
// saves the PointCloud *cloud* with clusters *cluster* as csv file.
void clusters_to_csv(const PointCloud &cloud);

#endif
