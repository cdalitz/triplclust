//
// pointcloud.cpp
//     Classes and functions for 3D points and clouds thereof.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2024-02-02
// License: see ../LICENSE
//

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

#include "kdtree/kdtree.hpp"
#include "pointcloud.h"
#include "util.h"

// a single 3D point
Point::Point(const std::vector<double> &point) {
  if (point.size() != 3) {
    throw std::invalid_argument("Point::Point(): point must be of dimension 3");
  }
  this->x = point[0];
  this->y = point[1];
  this->z = point[2];
}

Point::Point(const std::vector<double> &point,
             const std::set<size_t> &cluster_ids) {
  if (point.size() != 3) {
    throw std::invalid_argument("Point::Point(): point must be of dimension 3");
  }
  this->x = point[0];
  this->y = point[1];
  this->z = point[2];
  this->cluster_ids = cluster_ids;
}

Point::Point(double x, double y, double z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

Point::Point(double x, double y, double z,
             const std::set<size_t> &cluster_ids) {
  this->x = x;
  this->y = y;
  this->z = z;
  this->cluster_ids = cluster_ids;
}

Point::Point(double x, double y, double z, size_t index) {
  this->x = x;
  this->y = y;
  this->z = z;
  this->index = index;  // only used for chronological order
}


// representation of 3D point as std::vector.
std::vector<double> Point::as_vector() const {
  std::vector<double> point(3);
  point[0] = this->x;
  point[1] = this->y;
  point[2] = this->z;
  return point;
}

// Euclidean norm of the point
double Point::norm() const { return sqrt((x * x) + (y * y) + (z * z)); }

// squared euclidean norm of the point
double Point::squared_norm() const { return (x * x) + (y * y) + (z * z); }

Point &Point::operator=(const Point &other) {
  x = other.x;
  y = other.y;
  z = other.z;
  return *this;
}

bool Point::operator==(const Point &p) const {
  return (x == p.x && y == p.y && z == p.z);
}

// formatted output of the point
std::ostream &operator<<(std::ostream &strm, const Point &p) {
  return strm << p.x << " " << p.y << " " << p.z;
}

// vector addition
Point Point::operator+(const Point &p) const {
  Point ret;
  ret.x = this->x + p.x;
  ret.y = this->y + p.y;
  ret.z = this->z + p.z;
  return ret;
}

// vector subtraction
Point Point::operator-(const Point &p) const {
  Point ret;
  ret.x = this->x - p.x;
  ret.y = this->y - p.y;
  ret.z = this->z - p.z;
  return ret;
}

// scalar product (dot product)
double Point::operator*(const Point &p) const {
  return this->x * p.x + this->y * p.y + this->z * p.z;
}

// scalar division
Point Point::operator/(double c) const { return Point(x / c, y / c, z / c); }

// scalar multiplication
Point operator*(Point x, double c) {
  Point v(c * x.x, c * x.y, c * x.z);
  return v;
}
Point operator*(double c, Point x) {
  Point v(c * x.x, c * x.y, c * x.z);
  return v;
}

PointCloud::PointCloud() { this->points2d = false; }

void PointCloud::set2d(bool is2d) { this->points2d = is2d; }

bool PointCloud::is2d() const { return this->points2d; }


void PointCloud::setOrdered(bool ordered) {this->ordered=ordered;}
 
bool PointCloud::isOrdered() const { return this->ordered; }


// Split string *input* into substrings by *delimiter*. The result is
// returned in *result*
void split(const std::string &input, std::vector<std::string> &result,
           const char delimiter) {
  std::stringstream ss(input);
  std::string element;

  while (std::getline(ss, element, delimiter)) {
    result.push_back(element);
  }
}

//-------------------------------------------------------------------
// Load csv file.
// The csv file is split by *delimiter* and saved in *cloud*.
// Lines starting with '#' are ignored.
// If there are more than 3 columns all other are ignored and
// if there are two columns the PointCloud is set to 2D.
// Throws invalid_argument exception in case of problems.
//-------------------------------------------------------------------
void load_csv_file(const char *fname, PointCloud &cloud, const char delimiter,
                   size_t skip) {
  std::ifstream infile(fname);
  std::string line;
  std::vector<std::string> items;
  size_t count = 0, count2d = 0, skiped = 0, countpoints = 0;
  //size_t countOrdered = 0; //!
  if (infile.fail()) throw std::exception();
  for (size_t i = 0; i < skip; ++i) {
    // skip the header
    std::getline(infile, line, '\n');
    skiped++;
  }
  while (!infile.eof()) {
#ifdef WEBDEMO
    if (countpoints > 1000)
      throw std::length_error("Number of points limited to 1000 in demo mode");
#endif
    //std::getline(infile, line, '\n');
    std::getline(infile, line, '\n');
    count++;

    // skip comments and empty lines
    if (line[0] == '#' || line.empty() ||
        line.find_first_not_of("\n\r\t ") == std::string::npos)
      continue;

    countpoints++;
    Point point;
    split(line, items, delimiter);
    if (items.size() < 2) {
      std::ostringstream oss;
      oss << "row " << count + skiped << ": "
          << "To few columns!";
      throw std::invalid_argument(oss.str());
    } else if (items.size() == 2) {
      items.push_back("0");  // z=0 for 2D data (size=2)
      count2d++;
    }
    size_t column = 1;
    try {
      // create point
      point.x = stod(items[0].c_str());
      column++;
      point.y = stod(items[1].c_str());
      column++;
      point.z = stod(items[2].c_str());
      point.index = countpoints-1;
      cloud.push_back(point);
    } catch (const std::invalid_argument &e) {
      std::ostringstream oss;
      oss << "row " << count + skiped << " column " << column << ": "
          << e.what();
      throw std::invalid_argument(oss.str());
    }

    items.clear();
  }

  // check if the cloud is 2d or if a problem occurred
  if (count2d && count2d != cloud.size()) {
    throw std::invalid_argument("Mixed 2d and 3d points.");
  } else if (count2d) {
    cloud.set2d(true);
  }
}

//-------------------------------------------------------------------
// Smoothing of the PointCloud *cloud*.
// For every point the nearest neighbours in the radius *r* is searched
// and the centroid of this neighbours is computed. The result is
// returned in *result_cloud* and contains these centroids. The
// centroids are duplicated in the result cloud, so it has the same
// size and order as *cloud*.
//-------------------------------------------------------------------
void smoothen_cloud(const PointCloud &cloud, PointCloud &result_cloud,
                    double r) {
  Kdtree::KdNodeVector nodes;

  // If the smooth-radius is zero return the unsmoothed pointcloud
  if (r == 0) {
    result_cloud = cloud;
    return;
  }

  // build kdtree
  for (size_t i = 0; i < cloud.size(); ++i) {
    nodes.push_back(cloud[i].as_vector());
  }
  Kdtree::KdTree kdtree(&nodes);

  for (size_t i = 0; i < cloud.size(); ++i) {
    size_t result_size;
    Point new_point, point = cloud[i];
    Kdtree::KdNodeVector result;

    kdtree.range_nearest_neighbors(point.as_vector(), r, &result);
    result_size = result.size();

    // compute the centroid with mean
    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> z_list;

    for (Kdtree::KdNodeVector::iterator it = result.begin(); it != result.end();
         ++it) {
      x_list.push_back(it->point[0]);
      y_list.push_back(it->point[1]);
      z_list.push_back(it->point[2]);
    }

    new_point.x =
        std::accumulate(x_list.begin(), x_list.end(), 0.0) / result_size;

    new_point.y =
        std::accumulate(y_list.begin(), y_list.end(), 0.0) / result_size;

    new_point.z =
        std::accumulate(z_list.begin(), z_list.end(), 0.0) / result_size;

    new_point.index = point.index;
    result_cloud.push_back(new_point);
  }
  result_cloud.setOrdered(cloud.isOrdered());
}
