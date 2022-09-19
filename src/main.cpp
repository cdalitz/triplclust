//
// main.cpp
//     Main file for reference implementation of the TriplClust algorithm.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2019-04-02
// License: see ../LICENSE
//

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "cluster.h"
#include "dnn.h"
#include "graph.h"
#include "option.h"
#include "output.h"
#include "pointcloud.h"

// usage message
const char *usage =
    "Usage:\n"
    "\ttriplclust [options] <infile>\n"
    "Options (defaults in brackets):\n"
    "\t-r <radius>    radius for point smoothing [2dNN]\n"
    "\t               (can be numeric or multiple of dNN)\n"
    "\t-k <n>         number of neighbours in triplet creation [19]\n"
    "\t-n <n>         number of the best triplets to use [2]\n"
    "\t-a <alpha>     maximum value for the angle between the\n"
    "\t               triplet branches [0.03]\n"
    "\t-s <scale>     scalingfactor for clustering [0.33dNN]\n"
    "\t               (can be numeric or multiple of dNN)\n"
    "\t-t <dist>      best cluster distance [auto]\n"
    "\t               (can be numeric or 'auto')\n"
    "\t-m <n>         minimum number of triplets for a cluster [5]\n"
    "\t-dmax <n>      max gapwidth within a triplet [none]\n"
    "\t               (can be numeric, multiple of dNN or 'none')\n"
    "\t-link <method> linkage method for clustering [single]\n"
    "\t               (can be 'single', 'complete', 'average')\n"
    "\t-oprefix <prefix>\n"
    "\t               write result not to stdout, but to <prefix>.csv\n"
    "\t               and (if -gnuplot is set) to <prefix>.gnuplot\n"
    "\t-gnuplot       print result as a gnuplot command\n"
    "\t-delim <char>  single char delimiter for csv input [' ']\n"
    "\t-skip <n>      number of lines skipped at head of infile [0]\n"
    "\t-v             be verbose\n"
    "\t-vv            be more verbose and write debug trace files\n"
    "Version:\n"
    "\t1.3 from 2019-04-02";

int main(int argc, char **argv) {
  // parse commandline
  Opt opt_params;
  if (opt_params.parse_args(argc, argv) != 0) {
    std::cerr << usage << std::endl;
    return 1;
  }
  const char *infile_name = opt_params.get_ifname();
  const char *outfile_prefix = opt_params.get_ofprefix();
  int opt_verbose = opt_params.get_verbosity();

  // plausibility checks
  if (!infile_name) {
    std::cerr << "[Error] no infile given!\n" << usage << std::endl;
    return 1;
  }

  // load data
  PointCloud cloud_xyz;
  try {
    load_csv_file(infile_name, cloud_xyz, opt_params.get_delimiter(),
                  opt_params.get_skip());
  } catch (const std::invalid_argument &e) {
    std::cerr << "[Error] in file'" << infile_name << "': " << e.what()
              << std::endl;
    return 2;
  }
#ifdef WEBDEMO
  // maximum pointcloud size error for webdemo
  catch (const std::length_error &e) {
    std::cerr << "[Error] in file'" << infile_name << "': " << e.what()
              << std::endl;
    return 3;
  }
#endif
  catch (const std::exception &e) {
    std::cerr << "[Error] cannot read infile '" << infile_name << "'! "
              << e.what() << std::endl;
    return 2;
  }
  if (cloud_xyz.size() == 0) {
    std::cerr << "[Error] empty cloud in file '" << infile_name << "'"
              << std::endl
              << "maybe you used the wrong delimiter" << std::endl;
    return 2;
  }

  // compute characteristic length dnn if needed
  if (opt_params.needs_dnn()) {
    double dnn = std::sqrt(first_quartile(cloud_xyz));
    if (opt_verbose > 0) {
      std::cout << "[Info] computed dnn: " << dnn << std::endl;
    }
    opt_params.set_dnn(dnn);
    if (dnn == 0.0) {
      std::cerr << "[Error] dnn computed as zero. "
                << "Suggestion: remove doublets, e.g. with 'sort -u'"
                << std::endl;
      return 3;
    }
  }

  // Step 1) smoothing by position averaging of neighboring points
  PointCloud cloud_xyz_smooth;
  smoothen_cloud(cloud_xyz, cloud_xyz_smooth, opt_params.get_r());

  if (opt_verbose > 1) {
    bool rc;
    rc = cloud_to_csv(cloud_xyz_smooth);
    if (!rc)
      std::cerr << "[Error] can't write debug_smoothed.csv" << std::endl;
    rc = debug_gnuplot(cloud_xyz, cloud_xyz_smooth);
    if (!rc)
      std::cerr << "[Error] can't write debug_smoothed.gnuplot" << std::endl;
  }

  // Step 2) finding triplets of approximately collinear points
  std::vector<triplet> triplets;
  generate_triplets(cloud_xyz_smooth, triplets, opt_params.get_k(),
                    opt_params.get_n(), opt_params.get_a());

  // Step 3) single link hierarchical clustering of the triplets
  cluster_group cl_group;
  compute_hc(cloud_xyz_smooth, cl_group, triplets, opt_params.get_s(),
             opt_params.get_t(), opt_params.is_tauto(), opt_params.get_dmax(),
             opt_params.is_dmax(), opt_params.get_linkage(), opt_verbose);

  // Step 4) pruning by removal of small clusters ...
  cleanup_cluster_group(cl_group, opt_params.get_m(), opt_verbose);
  cluster_triplets_to_points(triplets, cl_group);
  // .. and (optionally) by splitting up clusters at gaps > dmax
  if (opt_params.is_dmax()) {
    cluster_group cleaned_up_cluster_group;
    for (cluster_group::iterator cl = cl_group.begin(); cl != cl_group.end();
         ++cl) {
      max_step(cleaned_up_cluster_group, *cl, cloud_xyz, opt_params.get_dmax(),
               opt_params.get_m() + 2);
    }
    cl_group = cleaned_up_cluster_group;
  }

  // store cluster labels in points
  add_clusters(cloud_xyz, cl_group, opt_params.is_gnuplot());

  if (outfile_prefix) {
    // redirect cout to outfile if requested
    std::streambuf *backup = std::cout.rdbuf();
    std::ofstream of;
    of.open((std::string(outfile_prefix) + ".csv").c_str());
    // replace the stream buffer from cout with the stream buffer from the
    // opened file, so that everythin printed to cout is printed to the file.
    std::cout.rdbuf(of.rdbuf());
    clusters_to_csv(cloud_xyz);
    of.close();
    if (opt_params.is_gnuplot()) {
      of.open((std::string(outfile_prefix) + ".gnuplot").c_str());
      clusters_to_gnuplot(cloud_xyz, cl_group);
      of.close();
    }
    // restore cout's default stream buffer
    std::cout.rdbuf(backup);
  } else if (opt_params.is_gnuplot()) {
    clusters_to_gnuplot(cloud_xyz, cl_group);
  } else {
    clusters_to_csv(cloud_xyz);
  }

  return 0;
}
