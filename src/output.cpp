//
// output.cpp
//     Functions for writing output files.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#include <stdint.h>
#include <cstdlib>
#include <iomanip>
#include <set>
#include <sstream>
#include <vector>

#include "output.h"

//-------------------------------------------------------------------
// Computation of cluster colour
// The colour is computed from the index of the cluster and is returned
// as hex value.
//-------------------------------------------------------------------
unsigned long compute_cluster_colour(size_t cluster_index) {
  const double r = (double)((cluster_index * 23) % 19) / 18.0;
  const double g = (double)((cluster_index * 23) % 7) / 6.0;
  const double b = (double)((cluster_index * 23) % 3) / 2.0;
  uint8_t r2 = (uint8_t)(r * 255);
  uint8_t g2 = (uint8_t)(g * 255);
  uint8_t b2 = (uint8_t)(b * 255);
  unsigned long rgb_hex = (r2 << 16) | (g2 << 8) | b2;

  return rgb_hex;
}

//-------------------------------------------------------------------
// Finds min and max Point of *cloud*
// the Points are returned in *min* and *max*.
//-------------------------------------------------------------------
void find_min_max_point(const PointCloud &cloud, Point &min, Point &max) {
  min = max = cloud[0];
  for (std::vector<Point>::const_iterator p = cloud.begin(); p != cloud.end();
       ++p) {
    if (min.x > p->x) {
      min.x = p->x;
    } else if (max.x < p->x) {
      max.x = p->x;
    }
    if (min.y > p->y) {
      min.y = p->y;
    } else if (max.y < p->y) {
      max.y = p->y;
    }
    if (min.z > p->z) {
      min.z = p->z;
    } else if (max.z < p->z) {
      max.z = p->z;
    }
  }
}

//-------------------------------------------------------------------
// saves smoothen cloud as gnuplot script.
// The script is saved as file 'debug_smoothed.gnuplot' in the current
// working directory. It contains the original PointCloud *cloud* in
// black and the smoothed PointCloud *cloud_smooth* in red.
// *fname* is the name of the new file. The function returns false if
// an error occurred.
//-------------------------------------------------------------------
bool debug_gnuplot(const PointCloud &cloud, const PointCloud &cloud_smooth,
                   const char *fname) {
  // saves cloud in gnuplot file
  std::ofstream of(fname);
  of << std::fixed;  // set float style
  bool is2d = cloud.is2d();
  if (!of.is_open()) {
    std::cerr << "[Error] could not save under '" << fname << std::endl;
    return false;
  }
  // find ranges for each axis
  Point min, max;
  find_min_max_point(cloud, min, max);
  // Write header
  if (!is2d) {
    // when max and min are the same, the script can't be ploted, so the range
    // must be changed
    if (max.x > min.x) {
      of << "set xrange [" << min.x << ":" << max.x << "]\n";
    } else {
      of << "set xrange [" << (min.x - 1.0) << ":" << (max.x + 1.0) << "]\n";
    }
    if (max.y > min.y) {
      of << "set yrange [" << min.y << ":" << max.y << "]\n";
    } else {
      of << "set yrange [" << (min.y - 1.0) << ":" << (max.y + 1.0) << "]\n";
    }
    if (max.z > min.z) {
      of << "set zrange [" << min.z << ":" << max.z << "]\n ";
    } else {
      of << "set zrange [" << (min.z - 1.0) << ":" << (max.z + 1.0) << "]\n ";
    }
    of << "splot ";
  } else {
    // if we have 2D data we use plot instead of splot. For plot the header is
    // not needed.
    of << "plot ";
  }

  of << "'-' with points lc 'black' title 'original', '-' with points lc "
        "'red' title 'smoothed'\n";
  for (PointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it) {
    of << it->x << " " << it->y;
    if (!is2d) of << " " << it->z;
    of << std::endl;
  }
  of << "e\n";

  for (PointCloud::const_iterator it = cloud_smooth.begin();
       it != cloud_smooth.end(); ++it) {
    of << it->x << " " << it->y;
    if (!is2d) of << " " << it->z;
    of << std::endl;
  }
  of << "e\npause mouse keypress\n";

  of.close();
  return true;
}

//-------------------------------------------------------------------
// saves a PointCloud *cloud* as csv file.
// The file is saved with the name *fname* under the current working
// directory. The function returns false if an error occurred.
//-------------------------------------------------------------------
bool cloud_to_csv(const PointCloud &cloud, const char *fname) {
  std::ofstream of(fname);
  of << std::fixed;  // set float style
  bool is2d = cloud.is2d();
  if (!of.is_open()) {
    std::cerr << "[Error] could not save under '" << fname << std::endl;
    return false;
  }
  of << "# x,y,z" << std::endl;
  for (PointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it) {
    of << it->x << "," << it->y;
    if (!is2d) of << "," << it->z << std::endl;
  }
  of.close();
  return true;
}

//-------------------------------------------------------------------
// prints gnuplot script to stdout.
// rgb colours are created with the function *compute_cluster_colour*.
// The points of *cloud* are printed with the corresponding cluster/colour.
//-------------------------------------------------------------------
void clusters_to_gnuplot(const PointCloud &cloud,
                         const std::vector<cluster_t> &clusters) {
  std::vector<Point> points = cloud;
  std::ostringstream pointstream, header, noise, clstream;
  std::string noiseheader = "";
  bool is2d = cloud.is2d();
  // set output format for floats
  pointstream << std::fixed;
  header << std::fixed;
  noise << std::fixed;
  // find ranges for each axis
  Point min, max;
  find_min_max_point(cloud, min, max);
  // Write header
  if (!is2d) {
    // when max and min are the same, the script can't be ploted, so the range
    // must be changed
    if (max.x > min.x) {
      header << "set xrange [" << min.x << ":" << max.x << "]\n";
    } else {
      header << "set xrange [" << (min.x - 1.0) << ":" << (max.x + 1.0)
             << "]\n";
    }
    if (max.y > min.y) {
      header << "set yrange [" << min.y << ":" << max.y << "]\n";
    } else {
      header << "set yrange [" << (min.y - 1.0) << ":" << (max.y + 1.0)
             << "]\n";
    }

    if (max.z > min.z) {
      header << "set zrange [" << min.z << ":" << max.z << "]\nsplot ";
    } else {
      header << "set zrange [" << (min.z - 1.0) << ":" << (max.z + 1.0)
             << "]\nsplot ";
    }
  } else {
    // if we have 2D data we use plot instead of splot. For plot the header is
    // not needed.
    header << "plot";
  }

  // iterate over clusters
  for (size_t cluster_index = 0; cluster_index < clusters.size();
       ++cluster_index) {
    const std::vector<size_t> &point_indices = clusters[cluster_index];
    // if there are no points in the cluster, it is only contained in an overlap
    // cluster
    if (point_indices.size() == 0) continue;
    // add cluster header
    unsigned long rgb_hex = compute_cluster_colour(cluster_index);
    clstream << " '-' with points lc '#" << std::hex << rgb_hex;
    std::set<size_t> cluster_ids = cloud[point_indices[0]].cluster_ids;
    if (cluster_ids.size() > 1) {
      clstream << "' title 'overlap ";
      for (std::set<size_t>::const_iterator clid = cluster_ids.begin();
           clid != cluster_ids.end(); ++clid) {
        if (clid != cluster_ids.begin()) clstream << ";";
        clstream << *clid;
      }
    } else {
      clstream << "' title 'curve " << *cluster_ids.begin();
    }
    clstream << "',";

    // add points to script
    for (std::vector<size_t>::const_iterator it = point_indices.begin();
         it != point_indices.end(); ++it) {
      const Point &point = cloud[*it];

      // remove current point from vector points
      for (std::vector<Point>::iterator p = points.begin(); p != points.end();
           p++) {
        if (*p == point) {
          points.erase(p);
          break;
        }
      }
      pointstream << point.x << " " << point.y;
      if (!is2d) pointstream << " " << point.z;
      pointstream << std::endl;
    }
    pointstream << "e" << std::endl;
  }
  clstream << std::endl;

  // plot all points red which are not clustered
  if (points.size() > 0) {
    noiseheader = " '-' with points lc 'red' title 'noise',";
    for (std::vector<Point>::iterator it = points.begin(); it != points.end();
         ++it) {
      noise << it->x << " " << it->y;
      if (!is2d) noise << " " << it->z;
      noise << std::endl;
    }
    noise << "e" << std::endl;
  }

  std::cout << header.str() << noiseheader << clstream.str() << noise.str()
            << pointstream.str() << "pause mouse keypress\n";
}

//-------------------------------------------------------------------
// saves the PointCloud *cloud* with clusters as csv file.
// The csv file has following form:
//    x,y,z,clusterid
// or 2D:
//    x,y,clusterid
//-------------------------------------------------------------------
void clusters_to_csv(const PointCloud &cloud) {
  bool is2d = cloud.is2d();
  std::cout << std::fixed
            << "# Comment: curveID -1 represents noise\n# x, y, z, curveID\n";

  for (PointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it) {
    std::cout << it->x << "," << it->y << ",";
    if (!is2d) std::cout << it->z << ",";
    if (it->cluster_ids.empty()) {
      // Noise
      std::cout << "-1\n";
    } else {
      for (std::set<size_t>::const_iterator it2 = it->cluster_ids.begin();
           it2 != it->cluster_ids.end(); ++it2) {
        if (it2 != it->cluster_ids.begin()) {
          std::cout << ";";
        }
        std::cout << *it2;
      }
      std::cout << std::endl;
    }
  }
}
