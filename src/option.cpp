//
// option.cpp
//     Class and functions for parsing and storing command line options.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2024-02-02
// License: see ../LICENSE
//

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "option.h"

// initialize default values
Opt::Opt() {
  this->infile_name = NULL;
  this->outfile_prefix = NULL;
  this->gnuplot = false;
  this->delimiter = ' ';
  this->skip = 0;
  this->verbose = 0;

  // neighbourship smoothing
  this->r = 2;
  this->rdnn = true;

  // triplet building
  this->k = 19;
  this->n = 2;
  this->a = 0.03;

  // triplet clustering
  this->s = 0.3;
  this->sdnn = true;
  this->t = 0;
  this->tauto = true;
  this->dmax = 0.0;
  this->isdmax = false;
  this->dmax_dnn = false;
  this->ordered = false;
  this->link = SINGLE;

  this->m = 5;
}

//-------------------------------------------------------------------
// parse commandline arguments.
// Parses the commandline arguments in *argv* and returns 0 if no error
// occurred. *argc* is the count off arguments.
//-------------------------------------------------------------------
int Opt::parse_args(int argc, char** argv) {
  // parse command line
  std::pair<double, bool> tmp;
  try {
    for (int i = 1; i < argc; i++) {
      if (0 == strcmp(argv[i], "-v")) {
        if (this->verbose < 1) this->verbose = 1;
      } else if (0 == strcmp(argv[i], "-vv")) {
        if (this->verbose < 2) this->verbose = 2;
      } else if (0 == strcmp(argv[i], "-s")) {
        ++i;
        if (i >= argc) {
          return 1;
        }
        tmp = this->parse_argument(argv[i]);
        this->s = tmp.first;
        this->sdnn = tmp.second;
      } else if (0 == strcmp(argv[i], "-r")) {
        ++i;
        if (i >= argc) {
          return 1;
        }
        tmp = this->parse_argument(argv[i]);
        this->r = tmp.first;
        this->rdnn = tmp.second;
      } else if (0 == strcmp(argv[i], "-k")) {
        ++i;
        if (i < argc) {
          this->k = (int)stod(argv[i]);
        } else {
          return 1;
        }
      } else if (0 == strcmp(argv[i], "-ordered")) {
        this->ordered = true;
      } else if (0 == strcmp(argv[i], "-n")) {
        ++i;
        if (i < argc) {
          this->n = (int)stod(argv[i]);
        } else {
          return 1;
        }
      } else if (0 == strcmp(argv[i], "-a")) {
        ++i;
        if (i < argc) {
          this->a = stod(argv[i]);
        } else {
          return 1;
        }
      } else if (0 == strcmp(argv[i], "-t")) {
        ++i;
        if (i < argc) {
          if (strcmp(argv[i], "auto") == 0 ||
              strcmp(argv[i], "automatic") == 0) {
            this->tauto = true;
          } else {
            this->t = stod(argv[i]);
            this->tauto = false;
          }
        } else {
          return 1;
        }
      } else if (0 == strcmp(argv[i], "-m")) {
        ++i;
        if (i < argc) {
          this->m = (int)stod(argv[i]);
        } else {
          return 1;
        }
      } else if (0 == strcmp(argv[i], "-delim")) {
        ++i;
        if (i < argc) {
          if (strlen(argv[i]) > 1) {
            std::cerr << "[Error] only a character as delimiter is allowed"
                      << std::endl;
            return 1;
          }
          this->delimiter = *argv[i];
        } else {
          return 1;
        }
      } else if (0 == strcmp(argv[i], "-dmax")) {
        ++i;
        if (i >= argc) {
          return 1;
        }
        if (strcmp(argv[i], "none") == 0) {
          this->isdmax = false;
        } else {
          tmp = this->parse_argument(argv[i]);
          this->dmax = tmp.first;
          this->dmax_dnn = tmp.second;
          this->isdmax = true;
        }
      } else if (0 == strcmp(argv[i], "-link")) {
        ++i;
        if (i >= argc) {
          return 1;
        }
        if (strcmp(argv[i], "single") == 0) {
          this->link = SINGLE;
        } else if (strcmp(argv[i], "complete") == 0) {
          this->link = COMPLETE;
        } else if (strcmp(argv[i], "average") == 0) {
          this->link = AVERAGE;
        } else {
          std::cerr << "[Error] " << argv[i] << " is not a valide option!"
                    << std::endl;
          return 1;
        }
      } else if (0 == strcmp(argv[i], "-skip")) {
        ++i;
        if (i < argc) {
          int tmp = atoi(argv[i]);
          if (tmp < 0) {
            std::cerr << "[Error] skip takes only positive integers. parameter "
                         "is ignored!"
                      << std::endl;
          } else {
            this->skip = (size_t)tmp;
          }
        } else {
          return 1;
        }
      } else if (0 == strcmp(argv[i], "-oprefix")) {
        if (i + 1 == argc) {
          std::cerr << "[Error] not enough parameters" << std::endl;
          return 1;
        } else if (argv[i + 1][0] == '-') {
          std::cerr << "[Error] please enter outfile name" << std::endl;
          return 1;
        }
        this->outfile_prefix = argv[++i];
      } else if (0 == strcmp(argv[i], "-gnuplot")) {
        this->gnuplot = true;
      } else if (argv[i][0] == '-') {
        return 1;
      } else {
        this->infile_name = argv[i];
      }
    }
  } catch (const std::invalid_argument &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}

//-------------------------------------------------------------------
// parses the argument string *str*.
// the result is returned in *result*. *result_dnn* stores if *result*
// depends on dnn. If *str* is not a number a invalid_argument exception
// is thrown.
//-------------------------------------------------------------------
std::pair<double, bool> Opt::parse_argument(const char* str) {
  double result = 0.0;
  bool dnn = false;
  char buff[4];
  size_t count = sscanf(str, "%lf%3s", &result, buff);
  if (count == 2) {
    if (std::strcmp("dnn", buff) && std::strcmp("dNN", buff))
      throw std::invalid_argument("not a number");
    dnn = true;
  } else if (count == 0) {
    throw std::invalid_argument("not a number");
  }
  return std::pair<double, bool>(result, dnn);
}

//-------------------------------------------------------------------
// compute attributes which depend on dnn.
// If r,s,dmax depend on dnn their new value will be computed.
//-------------------------------------------------------------------
void Opt::set_dnn(double dnn) {
  if (this->rdnn) {
    this->r *= dnn;
    if (this->verbose > 0) {
      std::cout << "[Info] computed smoothed radius: " << this->r << std::endl;
    }
  }
  if (this->sdnn) {
    this->s *= dnn;
    if (this->verbose > 0) {
      std::cout << "[Info] computed distance scale: " << this->s << std::endl;
    }
  }
  if (this->dmax_dnn) {
    this->dmax *= dnn;
    if (this->verbose > 0) {
      std::cout << "[Info] computed max gap: " << this->dmax << std::endl;
    }
  }
}

// read access functions
const char* Opt::get_ifname() { return this->infile_name; }
const char* Opt::get_ofprefix() { return this->outfile_prefix; }
bool Opt::needs_dnn() { return this->rdnn || this->sdnn || this->dmax_dnn; }
bool Opt::is_gnuplot() { return this->gnuplot; }
size_t Opt::get_skip() { return this->skip; }
char Opt::get_delimiter() { return this->delimiter; }
int Opt::get_verbosity() { return this->verbose; }
double Opt::get_r() { return this->r; }
size_t Opt::get_k() { return this->k; }
size_t Opt::get_n() { return this->n; }
double Opt::get_a() { return this->a; }
double Opt::get_s() { return this->s; }
bool Opt::is_tauto() { return this->tauto; }
double Opt::get_t() { return this->t; }
bool Opt::is_dmax() { return this->isdmax; }
double Opt::get_dmax() { return this->dmax; }
Linkage Opt::get_linkage() { return this->link; }
size_t Opt::get_m() { return this->m; }
bool Opt::get_ordered() {return this->ordered;}
