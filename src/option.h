//
// option.h
//     Class and functions for parsing and storing command line options.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#ifndef OPTION_H
#define OPTION_H
#include <cstddef>
#include <utility>

#include "util.h"

// class for handling all command line option
class Opt {
 private:
  char *infile_name, *outfile_prefix;
  // output as gnuplot
  bool gnuplot;
  // csv file delimiter
  char delimiter;
  // csv file with header
  size_t skip;
  // verbosity level
  int verbose;

  // neighbour distance for smoothing
  double r;
  bool rdnn;  // compute r with dnn

  // tested neighbours of triplet mid point
  size_t k;
  // max number of triplets to one mid point
  size_t n;
  // 1 - cos alpha, where alpha is the angle between the two triplet branches
  double a;

  // distance scale factor in metric
  double s;
  bool sdnn;  // compute s with dnn
  // threshold for cdist in clustering
  double t;
  bool tauto;  // auto generate t
  // maximum gap width
  double dmax;
  bool isdmax;    // dmax != none
  bool dmax_dnn;  // use dnn for dmax
  // linkage method for clustering
  Linkage link;

  // min number of triplets per cluster
  size_t m;

  std::pair<double, bool> parse_argument(const char *str);

 public:
  Opt();
  int parse_args(int argc, char **argv);
  // compute attributes which depend on dnn.
  void set_dnn(double dnn);

  // read access functions
  const char *get_ifname();
  // get outfile name
  const char *get_ofprefix();
  bool needs_dnn();
  bool is_gnuplot();
  char get_delimiter();
  size_t get_skip();
  int get_verbosity();
  double get_r();
  size_t get_k();
  size_t get_n();
  double get_a();
  double get_s();
  bool is_tauto();
  double get_t();
  bool is_dmax();
  double get_dmax();
  Linkage get_linkage();
  size_t get_m();
};

#endif
